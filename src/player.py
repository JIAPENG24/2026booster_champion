"""Player:单个机器人的控制 handle —— 用户可直接编辑这个类。

平台原语(set_velocity / kick / release_kick / request_mode / get_up)委托给注入的
``_backend``(framework 层);状态 property(pose / mode / is_fallen / penalty)从
``self.context`` 或 backend 读。

走位行为(walk_to / face_to / ensure_ready)也是 Player 方法:它们本质是"对本
球员下命令的动词",且未来加迟滞/避障需要的跨帧状态可直接挂 ``self``。纯坐标计算
(dist / angle_to / 球门坐标等)才放 utils/geom。

实例跨整场比赛存活;``self.context`` 每帧被框架覆写。用户想加自己的拐棍方法直接
在这里加。
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

from .framework.types import Context, Penalty, Pose2D
from .framework import debugdraw

from .param import *
from .utils.geom import (
    angle_to,
    clamp,
    dist,
    normalize_angle,
    opponent_goal,
    own_goal,
    own_goal_area_center,
)
from .utils.obstacles import collect_obstacles
from .utils.path_planner import plan_global_path

if TYPE_CHECKING:
    from .framework.config import SoccerConfig


__all__ = ["Player"]


_log = logging.getLogger(__name__)

def _heading_clearance(
    px: float, py: float, heading: float, obstacles: list,
) -> float:
    """用于局部避障。沿 (px,py) 朝 ``heading`` 的 lookahead 射线,到最近障碍的余量(dist - radius)。

    只考虑**前方**(投影 t>0)的障碍;身后 / 侧后的障碍不挡这个方向,跳过——否则贴近
    任何障碍(哪怕正后方)都会把所有方向的余量拉低,误判"全堵"。
    无障碍返回 inf;越大越空,负值表示会撞。
    """
    ux, uy = math.cos(heading), math.sin(heading)
    min_clear = math.inf
    for obs in obstacles:
        t = (obs.x - px) * ux + (obs.y - py) * uy
        if t <= 0.0:
            continue                      # 障碍在身后/侧后,不挡此方向
        if t > PLAN_LOOKAHEAD:
            t = PLAN_LOOKAHEAD
        nx, ny = px + ux * t, py + uy * t
        clear = math.hypot(obs.x - nx, obs.y - ny) - obs.radius
        if clear < min_clear:
            min_clear = clear
    return min_clear


def _behind_ball(
    ball_x: float, ball_y: float, aim: tuple[float, float], offset: float,
) -> tuple[float, float]:
    """用于计算站位。球在 球→``aim`` 连线上的"后方"点:从球沿【背离 aim】方向退 ``offset`` 米。

    追球走位目标用它:球落在机器人与 ``aim`` 之间,到位时天然对准 aim 方向。
    aim 与球重合(退化)时,方向不定,直接返回球位。
    """
    dx, dy = aim[0] - ball_x, aim[1] - ball_y
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return (ball_x, ball_y)
    ux, uy = dx / d, dy / d              # 球指向 aim 的单位向量
    return (ball_x - ux * offset, ball_y - uy * offset)   # 背离 aim 退 offset


class Player:
    """单个球员的 handle。可以在这里加入新的技术动作。
    """

    def __init__(
        self,
        player_id: int,
        config: "SoccerConfig",
        _backend: object | None,
    ) -> None:
        self.id: int = player_id
        self.config: "SoccerConfig" = config
        self._backend = _backend           # 机器人控制接口的包装，通常不用改
        self.context: Context | None = None

        # 当前高层动作名(仅供可视化/调试)。策略分派层每帧写入;有子状态的动作
        # (如 guard)在方法内部细化。可视化 pass 读它标注文字,见 main.py。
        self.action: str = "init"

        # SDK 缓存字段,框架后台会自动更新
        self._mode: str | None = None
        self._fall_down_state: str | None = None

        # 避障绕行侧记忆(跨帧;None=当前无绕行)
        self._avoid_side: float | None = None

        # 踢球迟滞状态(跨帧)
        self._kicking: bool = False

        # block/guard 的跨帧迟滞状态
        self._block_pressing: bool = False
        self._guard_threatened: bool = False
        # 门线撞球进门迟滞状态
        self._goal_line_push: bool = False
        # support 卡住后临时转 attack 的跨帧状态
        self._support_last_pos: tuple[float, float] | None = None
        self._support_stationary_since: float | None = None
        self._support_last_update_at: float | None = None
        self._support_stuck_frames: int = 0

        # 2.1-1: 追球拦截用球速追踪
        self._ball_track_prev: tuple[float, float] | None = None
        self._ball_track_time: float | None = None
        self._ball_vx: float = 0.0
        self._ball_vy: float = 0.0

    # ------------------------------------------------------------------
    # 状态读取
    # ------------------------------------------------------------------

    @property
    def is_kicking(self) -> bool:
        """当前是否处于踢球状态。"""
        return self._kicking

    @property
    def pose(self) -> Pose2D | None:
        ctx = self.context
        if ctx is None:
            return None
        robot = ctx.teammates.get(self.id)
        return None if robot is None else robot.pose

    @property
    def mode(self) -> str | None:
        if self._backend is not None:
            return self._backend.mode
        return self._mode

    @property
    def is_fallen(self) -> bool:
        return self.fall_down_state not in (None, "normal")

    @property
    def fall_down_state(self) -> str | None:
        if self._backend is not None:
            return getattr(self._backend, "fall_down_state", None)
        return self._fall_down_state

    @property
    def penalty(self) -> Penalty:
        ctx = self.context
        if ctx is None or ctx.game is None:
            return Penalty.NONE
        state = ctx.game.get_player_state(self.config.team_id, self.id)
        return Penalty.NONE if state is None else state.penalty

    @property
    def is_penalized(self) -> bool:
        return self.penalty != Penalty.NONE

    # ------------------------------------------------------------------
    # 底盘控制
    # ------------------------------------------------------------------

    def set_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        if self._backend is None:
            _log.debug(
                "player %d set_velocity vx=%.3f vy=%.3f vyaw=%.3f (no backend)",
                self.id, vx, vy, vyaw,
            )
            return
        self._backend.set_velocity(vx, vy, vyaw)

    def stop(self) -> None:
        self.release_kick()
        self.set_velocity(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # 踢球
    # ------------------------------------------------------------------

    def kick(
        self,
        kick_direction: float | None = None,
        power: float = KICK_POWER_DEFAULT,
    ) -> None:
        pose = self.pose
        if pose is None:
            _log.warning("player %d kick skipped: pose unknown", self.id)
            return

        ball = self.context.ball if self.context is not None else None
        if ball is None:
            _log.warning("player %d kick skipped: ball unknown", self.id)
            return

        if kick_direction is None:
            kick_plan = self.plan_kick()
            if kick_plan is None:
                _log.warning("player %d kick skipped: kick plan unavailable", self.id)
                return
            kick_direction, power = kick_plan

        if self._backend is None:
            _log.debug(
                "player %d kick ball=(%.3f, %.3f) dir=%.3f (no backend)",
                self.id, ball.x, ball.y, kick_direction,
            )
            return
        # 场地坐标 → 体坐标(用当前 pose)
        dx = ball.x - pose.x
        dy = ball.y - pose.y
        cos_t = math.cos(pose.theta)
        sin_t = math.sin(pose.theta)
        ball_x_body = dx * cos_t + dy * sin_t
        ball_y_body = -dx * sin_t + dy * cos_t
        kick_direction = normalize_angle(kick_direction)
        direction_body = normalize_angle(kick_direction - pose.theta)
        power_clamped = max(KICK_POWER_MIN, min(KICK_POWER_MAX, power))
        self._kicking = True
        self._backend.kick(direction_body, power_clamped, ball_x_body, ball_y_body)

    def _find_goalie(self) -> Pose2D | None:
        """找到对方守门员位置（距对方球门最近者）。"""
        ctx = self.context
        if ctx is None:
            return None
        gx, gy = opponent_goal(ctx)
        best, best_dist = None, math.inf
        for opp in ctx.opponents.values():
            if opp.pose is None:
                continue
            d = dist(opp.pose.x, opp.pose.y, gx, gy)
            if d < best_dist:
                best_dist, best = d, opp.pose
        return best

    def plan_kick(self) -> tuple[float, float] | None:
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None:
            return None

        half_goal = ctx.field.goal_width / 2.0
        goal_x = ctx.field.length / 2.0

        # 远距离解围:球在我方半场时大脚向中场边路
        if self._in_backfield():
            side = 1.0 if self.id % 2 == 0 else -1.0
            clear_target = (0.0, side * ctx.field.width * 0.3)
            kick_dir = angle_to(ball.x, ball.y, *clear_target)
            self._draw_kick_target(clear_target)
            return kick_dir, KICK_POWER_BACKFIELD

        # 守门员感知:对方守门员偏一侧时射另一侧
        goalie = self._find_goalie()
        if goalie is not None:
            if goalie.y > 0.2:
                kick_target = (goal_x, -half_goal + 0.2)
            elif goalie.y < -0.2:
                kick_target = (goal_x, half_goal - 0.2)
            else:
                side = 1.0 if self.id % 2 == 0 else -1.0
                kick_target = (goal_x, side * half_goal * 0.3)
        else:
            # 根据球的位置选择射门目标
            if ball.y > 0.5:
                kick_target = (goal_x, -half_goal + 0.2)
            elif ball.y < -0.5:
                kick_target = (goal_x, half_goal - 0.2)
            else:
                side = 1.0 if self.id % 2 == 0 else -1.0
                kick_target = (goal_x, side * half_goal * 0.3)

        kick_direction = angle_to(ball.x, ball.y, *kick_target)
        kick_power = KICK_POWER_DEFAULT

        # 角度偏差检查:与球门中心方向偏离 > 60° 时放弃射门
        goal_center_dir = angle_to(ball.x, ball.y, goal_x, 0.0)
        angle_deviation = abs(normalize_angle(kick_direction - goal_center_dir))
        if angle_deviation > math.radians(60):
            return None

        self._draw_kick_target(kick_target)
        return kick_direction, kick_power

    def _goal_target_for_direction(
        self, kick_direction: float,
    ) -> tuple[float, float]:
        """把射门方向投到对方门线上,用于可视化踢球目标。"""
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None:
            return (0.0, 0.0)

        dx = math.cos(kick_direction)
        if dx <= 1e-6:
            return opponent_goal(ctx)
        goal_x = ctx.field.length / 2.0
        t = max(0.0, (goal_x - ball.x) / dx)
        return (goal_x, ball.y + math.sin(kick_direction) * t)

    def _in_backfield(self) -> bool:
        """球在我方后场时,默认踢球加大力度。"""
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None:
            return False
        # own_penalty_edge_x = -ctx.field.length / 2.0 + ctx.field.penalty_area_length
        # return ball.x < own_penalty_edge_x
        return ball.x < 0

    def _draw_kick_target(self, target: tuple[float, float]) -> None:
        """以 X 标出 plan_kick 选择的踢球目标。"""
        from .framework import debugdraw

        x, y = target
        s = KICK_TARGET_MARK_SIZE_M
        debugdraw.line(
            [(x - s, y - s), (x + s, y + s)],
            rgb=(1.0, 0.0, 1.0), ns="kick_target",
        )
        debugdraw.line(
            [(x - s, y + s), (x + s, y - s)],
            rgb=(1.0, 0.0, 1.0), ns="kick_target",
        )

    def release_kick(self) -> None:
        self._kicking = False  # 清除踢球迟滞标志(取消方块显示)
        if self._backend is None:
            _log.debug("player %d release_kick (no backend)", self.id)
            return
        self._backend.release_kick()

    def kick_can_score(self, kick_direction: float) -> bool:
        """判断从当前球位按 ``kick_direction`` 踢,直线轨迹是否能进对方球门。
        """
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None:
            return False

        goal_x = ctx.field.length / 2.0
        dx = math.cos(kick_direction)
        dy = math.sin(kick_direction)
        if dx <= 1e-6:
            return False

        half_goal = ctx.field.goal_width / 2.0
        if half_goal <= 0.0:
            return False
        if ball.x >= goal_x:
            y_at_goal = ball.y
        else:
            t = (goal_x - ball.x) / dx
            if t < 0.0:
                return False
            y_at_goal = ball.y + dy * t
        return -half_goal <= y_at_goal <= half_goal

    # ------------------------------------------------------------------
    # 慢操作（同步接口异步调取，使得不阻塞）
    # ------------------------------------------------------------------

    def request_mode(self, mode: str) -> None:
        if self._backend is None:
            _log.debug("player %d request_mode -> %s (no backend)", self.id, mode)
            return
        self._backend.request_mode(mode)

    def get_up(self) -> None:
        if self._backend is None:
            _log.debug("player %d get_up (no backend)", self.id)
            return
        self._backend.get_up()

    # ------------------------------------------------------------------
    # 走位
    # ------------------------------------------------------------------

    def ensure_ready(self) -> bool:
        """活性自理:摔倒→起身,非 walk→切模式(都异步、不产生移动)。

        返回本帧是否可执行动作(True=已就绪)。
        """
        if self.is_fallen:
            self.get_up()
            return False
        if self.mode != "walk":
            self.request_mode("walk")
            return False
        return True

    def face_to(self, target_theta: float) -> None:
        """原地转向到目标朝向。"""
        if self.pose is None:
            self.stop()
            return
        err = normalize_angle(target_theta - self.pose.theta)
        self.set_velocity(0.0, 0.0, self._angular(err))

    def walk_to(
        self,
        target: tuple[float, float],
        *,
        face: float | None = None,
        avoid_ball: bool = False,
        avoid_robots: bool = False,
        arrive_dist: float = ARRIVE_DIST,
    ) -> bool:
        """走向目标点。返回是否已到达。

        避障(局部规划器,简化版 VFH):``avoid_ball`` / ``avoid_robots`` 开启时,把
        球 / 机器人 / 球门结构收集成圆形障碍,在机器人周围扫一圈候选方向,选"最朝
        目标 + 前方 ``PLAN_LOOKAHEAD`` 内不撞障碍"的方向前进。能处理多障碍、凹形
        结构和对称冲突,比单障碍绕行 / 势场排斥稳。

        行走两种模式:近距离全向,远距离转身-走。``face`` 指定到达/近距离时的朝向。
        """
        self.release_kick() # 踢球状态下，走位会被覆盖，先取消踢球状态
        pose = self.pose
        if pose is None:
            self.stop()
            return False

        tx, ty = target
        dx = tx - pose.x
        dy = ty - pose.y
        distance = math.hypot(dx, dy)

        if distance < arrive_dist:
            # 已到达:按需转到目标朝向
            if face is not None:
                err = normalize_angle(face - pose.theta)
                if abs(err) > 0.1:
                    self.set_velocity(0.0, 0.0, self._angular(err))
                else:
                    self.stop()
            else:
                self.stop()
            return True

        # 规划:默认全局 A*,找不到路时退回旧局部规划。
        goal_dir = math.atan2(dy, dx)
        planned_path: list[tuple[float, float]] | None = None
        waypoint: tuple[float, float] | None = None
        if (avoid_ball or avoid_robots) and self.context is not None:
            obstacles = collect_obstacles(
                self.context, self.id,
                ball=avoid_ball, robots=avoid_robots,
                goals=(avoid_ball or avoid_robots),
            )
            if USE_GLOBAL_PATH_PLANNER:
                planned_path = plan_global_path(
                    self.context,
                    (pose.x, pose.y),
                    (tx, ty),
                    obstacles,
                )
                if planned_path is not None:
                    waypoint = self._path_waypoint(pose, planned_path)
                    heading = angle_to(pose.x, pose.y, waypoint[0], waypoint[1])
                else:
                    heading = self._plan_heading(pose, goal_dir, obstacles)
            else:
                heading = self._plan_heading(pose, goal_dir, obstacles)
        else:
            heading = goal_dir

        # 可视化:目标点(绿)、到目标连线(灰)、规划朝向(黄箭头)、
        # 前方探测射线(青,长度=lookahead;规划器"看"的范围,无 path/途径点概念)
        from .framework import debugdraw
        debugdraw.point(tx, ty, rgb=(0.0, 1.0, 0.0), scale=0.15, ns="target")
        debugdraw.line([(pose.x, pose.y), (tx, ty)], rgb=(0.4, 0.4, 0.4), ns="to_target")
        if planned_path is not None and len(planned_path) >= 2:
            debugdraw.line(planned_path, rgb=(0.2, 0.8, 1.0), ns="global_path")
        if waypoint is not None:
            debugdraw.point(
                waypoint[0], waypoint[1],
                rgb=(0.2, 0.8, 1.0), scale=0.12, ns="global_waypoint",
            )
        debugdraw.arrow(
            pose.x, pose.y,
            pose.x + math.cos(heading) * 0.6, pose.y + math.sin(heading) * 0.6,
            rgb=(1.0, 1.0, 0.0), ns="heading",
        )
        debugdraw.line(
            [(pose.x, pose.y),
             (pose.x + math.cos(heading) * PLAN_LOOKAHEAD,
              pose.y + math.sin(heading) * PLAN_LOOKAHEAD)],
            rgb=(0.0, 0.8, 0.8), ns="lookahead",
        )

        if distance <= OMNI_DIST:
            # 近距离:全向行走,沿 heading 平移,同时转向 face
            wdx, wdy = math.cos(heading) * distance, math.sin(heading) * distance
            cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
            vx = LINEAR_GAIN * (wdx * cos_t + wdy * sin_t)
            vy = LINEAR_GAIN * (-wdx * sin_t + wdy * cos_t)
            speed = math.hypot(vx, vy)
            if speed > MAX_LINEAR:
                vx *= MAX_LINEAR / speed
                vy *= MAX_LINEAR / speed
            vyaw = (
                self._angular(normalize_angle(face - pose.theta))
                if face is not None else 0.0
            )
            self.set_velocity(vx, vy, vyaw)
        else:
            # 远距离:转身-走-转身,朝 heading
            angle_err = normalize_angle(heading - pose.theta)
            if abs(angle_err) > TURN_THRESHOLD:
                self.set_velocity(0.0, 0.0, self._angular(angle_err))
            else:
                vx = clamp(
                    LINEAR_GAIN * distance * math.cos(angle_err), 0.0, MAX_LINEAR,
                )
                self.set_velocity(vx, 0.0, self._angular(angle_err))
        return False

    def _path_waypoint(
        self, pose: Pose2D, path: list[tuple[float, float]],
    ) -> tuple[float, float]:
        """Pick a short lookahead waypoint from a planned global path."""
        if not path:
            return (pose.x, pose.y)
        prev = (pose.x, pose.y)
        points = path[1:] if len(path) > 1 else path
        for point in points:
            seg_len = dist(prev[0], prev[1], point[0], point[1])
            if seg_len >= GLOBAL_PATH_LOOKAHEAD_M:
                ratio = GLOBAL_PATH_LOOKAHEAD_M / max(seg_len, 1e-6)
                return (
                    prev[0] + (point[0] - prev[0]) * ratio,
                    prev[1] + (point[1] - prev[1]) * ratio,
                )
            prev = point
        return path[-1]

    def _plan_heading(
        self, pose: Pose2D, goal_dir: float, obstacles: list,
    ) -> float:
        """扫候选方向,选最朝目标且前方够空的方向;都不够空时返回最空的那个。

        候选按偏离目标方向 ``|offset|`` 从小到大试;先试哪一侧由 player_id 奇偶决定,
        用来打破两人同侧避让的对称。
        """
        sign_first = 1.0 if self.id % 2 == 0 else -1.0
        best_h = goal_dir
        best_clear = -math.inf

        offsets = [0.0]
        k = 1
        while k * PLAN_STEP <= PLAN_MAX_OFFSET + 1e-9:
            offsets.append(sign_first * k * PLAN_STEP)
            offsets.append(-sign_first * k * PLAN_STEP)
            k += 1

        for off in offsets:
            h = goal_dir + off
            clear = _heading_clearance(pose.x, pose.y, h, obstacles)
            if clear >= PLAN_CLEARANCE:
                return h
            if clear > best_clear:
                best_clear, best_h = clear, h
        return best_h

    # ------------------------------------------------------------------
    # 高层动作(策略在 main.py 里直接调这些)
    # ------------------------------------------------------------------


    def block_path_projection(
        self, opponent_id: int,
    ) -> tuple[float, float, float, float] | None:
        """自己到 对手→球 线段的垂足:返回 (x, y, 垂距, 线段参数 t)。"""
        ctx = self.context
        pose = self.pose
        ball = ctx.ball if ctx is not None else None
        opponent = ctx.opponents.get(opponent_id) if ctx is not None else None
        if ctx is None or pose is None or ball is None or opponent is None:
            return None
        if opponent.pose is None:
            return None

        ax, ay = opponent.pose.x, opponent.pose.y
        bx, by = ball.x, ball.y
        vx, vy = bx - ax, by - ay
        length2 = vx * vx + vy * vy
        if length2 < 1e-6:
            return None

        raw_t = ((pose.x - ax) * vx + (pose.y - ay) * vy) / length2
        t = clamp(raw_t, 0.0, 1.0)
        tx = ax + vx * t
        ty = ay + vy * t
        return tx, ty, dist(pose.x, pose.y, tx, ty), raw_t

    def pass_to(self, teammate_id: int) -> bool:
        """传球给指定队友。返回是否成功发出。"""
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        mate = ctx.teammates.get(teammate_id) if ctx is not None else None
        if ball is None or mate is None or mate.pose is None or self.pose is None:
            return False
        kick_dir = angle_to(ball.x, ball.y, mate.pose.x, mate.pose.y)
        dist_to_mate = dist(ball.x, ball.y, mate.pose.x, mate.pose.y)
        power = clamp(dist_to_mate * 1.5, 2.0, 6.0)
        self.kick(kick_dir, power)
        self.action = f"pass_to_{teammate_id}"
        return True

    def _pick_dribble_direction(self, direct_dir: float) -> float | None:
        """检查直接射门方向是否有对手挡路,有则选变向方向,无则返回 None。"""
        ctx = self.context
        ball = ctx.ball if ctx is not None else None
        if ctx is None or ball is None:
            return None
        pose = self.pose
        if pose is None:
            return None

        # 收集 DRIBBLE_SCAN_RADIUS 内的对手
        threats = []
        for oid, opp in ctx.opponents.items():
            if opp.pose is None:
                continue
            od = dist(pose.x, pose.y, opp.pose.x, opp.pose.y)
            if od < DRIBBLE_SCAN_RADIUS:
                threats.append(opp.pose)

        if not threats:
            return None

        # 对手在直接方向的正前方投影(沿 direct_dir 方向)
        dx = math.cos(direct_dir)
        dy = math.sin(direct_dir)
        blocked = False
        for t in threats:
            tx, ty = t.x - pose.x, t.y - pose.y
            proj = tx * dx + ty * dy
            lateral = abs(-tx * dy + ty * dx)
            if proj > 0.0 and lateral < 0.5:
                blocked = True
                break

        if not blocked:
            return None

        # 被堵:扫描备选方向,选最朝 goal + 净空足够的
        best_h = direct_dir
        best_clear = -math.inf
        for offset in range(1, 8):
            for sign in (1.0, -1.0):
                h = direct_dir + sign * offset * DRIBBLE_SCAN_STEP
                h = normalize_angle(h)
                min_clear = math.inf
                for t in threats:
                    tx, ty = t.x - pose.x, t.y - pose.y
                    proj = math.cos(h) * tx + math.sin(h) * ty
                    lat = abs(-math.sin(h) * tx + math.cos(h) * ty)
                    if proj > 0.0 and lat < min_clear:
                        min_clear = lat
                if min_clear >= DRIBBLE_CLEARANCE:
                    return h
                if min_clear > best_clear:
                    best_clear = min_clear
                    best_h = h
        return best_h if best_h != direct_dir else None

    def attack(self, kick_target: tuple[float, float] | None = None) -> None:
        """追球射门。角度太差时会尝试传球,禁区内轻推。"""
        ball = self.context.ball if self.context is not None else None
        if ball is None or self.pose is None:
            self.stop()
            return
        if kick_target is None:
            kick_target = opponent_goal(self.context)

        # 更新球速追踪(用于拦截预测)
        if self._ball_track_prev is not None and self._ball_track_time is not None:
            dt = ball.last_seen_at - self._ball_track_time
            if 0.001 < dt < 1.0:
                self._ball_vx = (ball.x - self._ball_track_prev[0]) / dt
                self._ball_vy = (ball.y - self._ball_track_prev[1]) / dt
        self._ball_track_prev = (ball.x, ball.y)
        self._ball_track_time = ball.last_seen_at

        d = dist(self.pose.x, self.pose.y, ball.x, ball.y)
        self._kicking = d <= (KICK_EXIT_M if self._kicking else KICK_ENTER_M)
        if self._kicking:
            kick_plan = self.plan_kick()
            if kick_plan is None:
                ctx = self.context
                if ctx is not None:
                    best_mate = None
                    best_score = -999
                    for pid, mate in ctx.teammates.items():
                        if pid == self.id or mate.pose is None:
                            continue
                        if mate.pose.x > best_score:
                            best_score = mate.pose.x
                            best_mate = pid
                    if best_mate is not None and self.pass_to(best_mate):
                        return
                self.stop()
                return
            kick_direction, kick_power = kick_plan
            # 禁区内轻推
            if ball.x > 4.0:
                kick_power = clamp(kick_power * 0.5, 2.0, 3.5)
            # P2-11: 带球变向 - 检查前方是否有对手挡路
            dribble_dir = self._pick_dribble_direction(kick_direction)
            if dribble_dir is not None:
                kick_direction = dribble_dir
                kick_power = DRIBBLE_POWER
                self.action = "dribble"
            # 角度太差且有人可传时传球
            if dribble_dir is None and not self.kick_can_score(kick_direction):
                ctx = self.context
                if ctx is not None:
                    best_mate = None
                    best_score = -999
                    for pid, mate in ctx.teammates.items():
                        if pid == self.id or mate.pose is None:
                            continue
                        if mate.pose.x > best_score:
                            best_score = mate.pose.x
                            best_mate = pid
                    if best_mate is not None and self.pass_to(best_mate):
                        return
            self.kick(kick_direction, kick_power)
        else:
            self.release_kick()
            # 远距离时走向预测位拦截
            if d > KICK_ENTER_M * 2.0:
                pred_x = ball.x + self._ball_vx * 0.5
                pred_y = ball.y + self._ball_vy * 0.5
                walk_target = _behind_ball(pred_x, pred_y, kick_target, CHASE_BEHIND_M)
            else:
                walk_target = _behind_ball(ball.x, ball.y, kick_target, CHASE_BEHIND_M)
            self.walk_to(walk_target)

    def guard(self) -> None:
        """守门:三模式——HOME(门线跟球)、RUSH(前冲封堵)、SWEEP(前提清道夫)。"""
        ctx = self.context
        if ctx is None or self.pose is None:
            self.action = "guard:stop"
            self.stop()
            return

        ball = ctx.ball
        gx, gy = own_goal(ctx)
        home_x = own_goal_area_center(ctx)[0]

        # 球不可见时回 HOME
        if ball is None:
            target = (home_x, 0.0)
            mode = "home"
        elif ball.x > GUARD_RUSH_X:
            # RUSH: 球逼近禁区,前冲封射门角度
            ratio = min(
                (ball.x - GUARD_RUSH_X) / (ctx.field.length / 2.0 - GUARD_RUSH_X),
                GUARD_RUSH_MAX_RATIO,
            )
            tx = gx + ratio * (ball.x - gx) * 0.5
            max_y = ctx.field.goal_width / 2.0 - 0.3
            ty = clamp(ball.y * 0.6, -max_y, max_y)
            target = (tx, ty)
            mode = "rush"
        elif ball.x < GUARD_SWEEP_X:
            # SWEEP: 球在对方半场,前提清道夫
            tx = GUARD_SWEEP_X_POS
            max_y = ctx.field.goal_width / 2.0 - 0.3
            ty = clamp(ball.y * 0.4, -max_y, max_y)
            target = (tx, ty)
            mode = "sweep"
        else:
            # HOME: 门线横向跟球
            max_y = ctx.field.goal_width / 2.0 - 0.3
            target_y = clamp(ball.y * 0.5, -max_y, max_y)
            if ball.x > 0:
                target_y *= 0.6
            target = (home_x, target_y)
            mode = "home"

        face = 0.0
        if GUARD_FACE_BALL and ball is not None:
            face = angle_to(self.pose.x, self.pose.y, ball.x, ball.y)

        self.action = f"guard:{mode}"
        debugdraw.point(
            target[0], target[1], rgb=(0.0, 0.6, 1.0), scale=0.2, ns="guard_home",
        )
        self.walk_to(target, face=face, avoid_ball=True, avoid_robots=True)

    def support(self, mark_opponent_id: int | None = None) -> None:
        """支援:可指定盯防对手,否则根据球位置动态站位。"""
        ctx = self.context
        if ctx is None:
            self.stop()
            return

        ball = ctx.ball

        # 盯人模式:站在最危险对手和我方球门之间
        if mark_opponent_id is not None:
            opp = ctx.opponents.get(mark_opponent_id)
            if opp is not None and opp.pose is not None and ball is not None:
                gx, gy = own_goal(ctx)
                ox, oy = opp.pose.x, opp.pose.y
                # 对手→球门连线中点偏球门侧 (70% toward goal)
                tx = (ox + gx) / 2.0 * 0.7 + gx * 0.3
                ty = (oy + gy) / 2.0
                half_l = ctx.field.length / 2.0
                half_w = ctx.field.width / 2.0 - 0.3
                tx = clamp(tx, -half_l + 0.3, half_l)
                ty = clamp(ty, -half_w, half_w)
                self.move_to_position((tx, ty))
                self.action = "mark"
                # 盯人模式下也做卡住检测
                if self.pose is not None:
                    if self._support_last_pos is not None:
                        moved = dist(
                            self.pose.x, self.pose.y,
                            self._support_last_pos[0], self._support_last_pos[1],
                        )
                        if moved < 0.05:
                            self._support_stuck_frames += 1
                        else:
                            self._support_stuck_frames = 0
                    self._support_last_pos = (self.pose.x, self.pose.y)
                    if self._support_stuck_frames > SUPPORT_STUCK_FRAMES_MAX:
                        self._support_stuck_frames = 0
                        self.action = "support_stuck_attack"
                        self.attack()
                return

        # 备用:原逻辑
        if ball is None:
            self.move_to_position(own_goal_area_center(ctx))
            return

        if ball.x < -3.0:
            dist_factor = 1.5
        elif ball.x < 0:
            dist_factor = 2.5
        elif ball.x < 3.0:
            dist_factor = 3.5
        else:
            dist_factor = 4.0

        gx, gy = own_goal(ctx)
        bx, by = ball.x, ball.y
        dx, dy = gx - bx, gy - by
        d = math.hypot(dx, dy)
        if d < 1e-6:
            ux, uy = -1.0, 0.0
        else:
            ux, uy = dx / d, dy / d
        along = min(dist_factor, d)
        tx = bx + ux * along
        ty = by + uy * along

        half_l = ctx.field.length / 2.0
        half_w = ctx.field.width / 2.0 - 0.3
        tx = clamp(tx, -half_l + 0.3, half_l)
        ty = clamp(ty, -half_w, half_w)
        self.move_to_position((tx, ty))

        # 卡住检测: 长时间不动则临时 attack
        if self.pose is not None:
            if self._support_last_pos is not None:
                moved = dist(
                    self.pose.x, self.pose.y,
                    self._support_last_pos[0], self._support_last_pos[1],
                )
                if moved < 0.05:
                    self._support_stuck_frames += 1
                else:
                    self._support_stuck_frames = 0
            self._support_last_pos = (self.pose.x, self.pose.y)
            if self._support_stuck_frames > SUPPORT_STUCK_FRAMES_MAX:
                self._support_stuck_frames = 0
                self.action = "support_stuck_attack"
                self.attack()

    def take_kickoff(self, kick_target: tuple[float, float] | None = None) -> None:
        """我方开球/重开:未就位则绕到球后待命(避球不碰),就位后接近并踢。"""
        ball = self.context.ball if self.context is not None else None
        if ball is None or self.pose is None:
            self.stop()
            return
        if kick_target is None:
            kick_target = opponent_goal(self.context)
        kick_dir = angle_to(ball.x, ball.y, *kick_target)
        cos_k, sin_k = math.cos(kick_dir), math.sin(kick_dir)
        rel_x, rel_y = self.pose.x - ball.x, self.pose.y - ball.y
        behind = rel_x * cos_k + rel_y * sin_k          # <0 在球后方(己方侧)
        lateral = abs(-rel_x * sin_k + rel_y * cos_k)
        if behind > KICKOFF_FRONT_MARGIN or lateral > KICKOFF_LATERAL_TOL:
            stage = (
                ball.x - cos_k * KICKOFF_STAGE_M,
                ball.y - sin_k * KICKOFF_STAGE_M,
            )
            self.release_kick()
            self.walk_to(stage, face=kick_dir, avoid_ball=True)
        else:
            self.attack(kick_target)

    def keep_away(self) -> None:
        """控球拖延:走向己方角落安全区,被逼抢则传最远的队友。"""
        ctx = self.context
        pose = self.pose
        ball = ctx.ball if ctx is not None else None
        if ctx is None or pose is None or ball is None:
            self.stop()
            return

        d = dist(pose.x, pose.y, ball.x, ball.y)

        # 有球权:带向己方角落或传球
        if d < KICK_ENTER_M * 1.5:
            # 最近对手距离
            min_opp_dist = math.inf
            for opp in ctx.opponents.values():
                if opp.pose is None:
                    continue
                od = dist(pose.x, pose.y, opp.pose.x, opp.pose.y)
                if od < min_opp_dist:
                    min_opp_dist = od

            if min_opp_dist < KEEP_AWAY_PRESSURE_DIST:
                # 被逼抢:传最远的队友
                best_mate = None
                best_dist = -1.0
                for pid, mate in ctx.teammates.items():
                    if pid == self.id or mate.pose is None:
                        continue
                    md = dist(pose.x, pose.y, mate.pose.x, mate.pose.y)
                    if md > best_dist:
                        best_dist = md
                        best_mate = pid
                if best_mate is not None and self.pass_to(best_mate):
                    return

            # 带向己方角落
            half_l = ctx.field.length / 2.0
            half_w = ctx.field.width / 2.0
            corner_x = -half_l + KEEP_AWAY_CORNER_MARGIN
            corner_y = half_w - KEEP_AWAY_CORNER_MARGIN if self.id % 2 == 0 else -half_w + KEEP_AWAY_CORNER_MARGIN
            face = angle_to(pose.x, pose.y, corner_x, corner_y)
            self.release_kick()
            self.walk_to(
                (corner_x, corner_y), face=face,
                avoid_ball=True, avoid_robots=True,
            )
            return

        # 无球:走向球侧方接应
        side = 1.0 if self.id % 2 == 0 else -1.0
        support_x = clamp(ball.x - 1.5, -ctx.field.length / 2.0 + 0.3, ctx.field.length / 2.0)
        support_y = clamp(ball.y + side * 2.0, -ctx.field.width / 2.0 + 0.3, ctx.field.width / 2.0 - 0.3)
        self.release_kick()
        self.walk_to(
            (support_x, support_y),
            avoid_ball=True, avoid_robots=True,
        )

    def move_to_position(self, target: tuple[float, float] | None) -> None:
        """走到站位点(支援/防守/避让),面向球,避球+避机器人。"""
        if target is None:
            self.stop()
            return
        face = None
        ball = self.context.ball if self.context is not None else None
        if ball is not None and self.pose is not None:
            face = angle_to(self.pose.x, self.pose.y, ball.x, ball.y)
        self.release_kick()
        self.walk_to(target, face=face, avoid_ball=True, avoid_robots=True)

    @staticmethod
    def _angular(err: float) -> float:
        return clamp(ANGULAR_GAIN * err, -MAX_ANGULAR, MAX_ANGULAR)
