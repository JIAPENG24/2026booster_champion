"""SoccerSim 策略入口 —— 比赛策略主逻辑都在这里,改打法就改这个文件。

结构(由浅入深):
- main.py(本文件):比赛策略。play() 按 Phase 状态机分派到 _act_*;各 _act_* 选出
  attacker(离球最近)并直接调 player 动作。
- player.py:Player 控制 handle + 高层动作(attack / take_kickoff /
  move_to_position / walk_to);想加拐棍/技术动作直接改它。
- utils/:走位/几何/避障工具(opponent_goal / dist / angle_to ...)。
- framework/:平台管线,用户不改。

改打法主要改本文件:Phase 状态机、各 _act_* 行为、站位公式。
"""

from __future__ import annotations

import logging
import math
from enum import Enum

from booster_agent_framework import AgentBase

from .framework.agent import SoccerAgentMixin
from .framework.types import KICKING_TEAM_NONE, Context, GameState, SetPlay
from .param import *
from .player import Player
from .utils import angle_to, clamp, dist, opponent_goal, own_goal


_log = logging.getLogger(__name__)


# ======================================================================
# Phase 状态机 —— 比赛阶段分类
# ======================================================================


class Phase(Enum):
    """比赛阶段。顶层状态机,决定当前是正常拼抢/开球/定位球/准备/停止。"""
    NORMAL = "normal"              # PLAYING 正常拼抢
    OUR_KICKOFF = "our_kickoff"    # 我方开球(SET+PLAYING 初期,take_kickoff)
    OPP_KICKOFF = "opp_kickoff"    # 对方开球(避让)
    OUR_SET_PLAY = "our_set_play"  # 我方定位球(任意球/角球/球门球)
    OPP_SET_PLAY = "opp_set_play"  # 对方定位球(避让)
    READY = "ready"                # READY 走位
    STOPPED = "stopped"            # SET(非开球重开) / INITIAL / FINISHED / stopped


def get_phase(context: Context) -> Phase:
    """根据裁判机状态判断当前比赛阶段。"""
    g = context.game
    if g is None:
        return Phase.STOPPED

    state = g.state

    # READY:走 ready 位
    if state == GameState.READY:
        return Phase.READY

    # PLAYING:正常拼抢 or 开球/定位球执行中
    if state == GameState.PLAYING and not g.stopped:
        # 定位球:set_play != NONE,kicking_team 指示哪方
        if g.set_play != SetPlay.NONE and g.kicking_team != KICKING_TEAM_NONE:
            our_team = context.team_id
            if g.kicking_team == our_team:
                return Phase.OUR_SET_PLAY
            else:
                return Phase.OPP_SET_PLAY

        # 开球:secondary_time > 0(倒计时窗口),kicking_team 指示哪方
        if g.secondary_time > 0 and g.kicking_team != KICKING_TEAM_NONE:
            our_team = context.team_id
            if g.kicking_team == our_team:
                return Phase.OUR_KICKOFF
            else:
                return Phase.OPP_KICKOFF

        # 正常拼抢
        return Phase.NORMAL

    # SET / INITIAL / FINISHED / stopped:站定
    return Phase.STOPPED

def get_set_play_type(context: Context) -> SetPlay:
    """当前生效的定位球类型;无定位球(或无裁判机数据)时返回 ``SetPlay.NONE``。

    直接读裁判机的 ``set_play`` 字段,不区分是哪方主罚 —— 哪方由 :func:`get_phase`
    (OUR_SET_PLAY / OPP_SET_PLAY)判定。这里只回答"是什么类型的定位球"。

    共 7 种可能返回值(见 framework.types.SetPlay):
    - ``NONE``:无定位球(正常比赛/开球等)
    - ``DIRECT_FREE_KICK``:直接任意球(可直接射门得分)
    - ``INDIRECT_FREE_KICK``:间接任意球(须先触碰他人才能进球)
    - ``PENALTY_KICK``:点球
    - ``THROW_IN``:界外球(踢入)
    - ``GOAL_KICK``:球门球
    - ``CORNER_KICK``:角球
    """
    g = context.game
    if g is None:
        return SetPlay.NONE
    return g.set_play


# ======================================================================
# Agent 入口
# ======================================================================


class SoccerSimAgent(SoccerAgentMixin, AgentBase):
    """3v3 SoccerSim agent。"""

    player_class = Player

    def init_store(self, store) -> None:
        _log.info("init_store called")
        store.prev_phase = None       # 上一帧 phase,用于检测 phase 跳变(边沿)
        store.cur_phase = None
        store.kickoff_taker = None    # 锁定的开球主罚球员 id(每次进入开球时重选)
        store.normal_attacker = None
        store.prev_ball_x = None      # 球速预测用
        store.prev_ball_y = None
        store.prev_ball_time = None

        # P2-13: 角色交换 — 停滞检测
        store.attacker_stuck_counter = 0
        store.attacker_prev_dist = None

        # P2-14: Keep-away
        store.keep_away_active = False

    @staticmethod
    def play(context: Context, players: list[Player], store) -> None:
        phase = get_phase(context)
        store.prev_phase = store.cur_phase
        store.cur_phase = phase

        # 画可视化(每帧)
        _analyze_and_draw(context, players, store)

        # 当前 phase 以 label 画在场外。
        from .framework import debugdraw
        g = context.game
        game_state = g.state.value if g is not None else "none"
        set_play = g.set_play.value if g is not None else "none"
        secondary_time = g.secondary_time if g is not None else 0.0
        debugdraw.text(
            0.0, context.field.width / 2.0 + 0.2,
            f"phase={phase.value} state={game_state} set={set_play} secondary={secondary_time:.1f}",
            rgb=(1.0, 1.0, 0.0), ns="phase",
        )

        # 活性自理 + 过滤出本帧可行动的球员。
        # ensure_ready:摔倒起身 / 切 walk 模式(异步,不产生移动);被罚下的也做,
        # 这样解罚后能立刻投入。被罚下或未就绪的不参与分派(也不进角色分配,避免把
        # 动不了的人选成 attacker 导致该帧无人进攻)。
        active: list[Player] = []
        for p in players:
            ready = p.ensure_ready()
            if p.is_penalized:
                p.action = "penalized"     # 罚下:可起身/切模式,但不能移动
                p.stop()
            elif not ready:
                p.action = "fallen" if p.is_fallen else "switching_mode"
            elif p.pose is None:
                p.action = "no_pose"       # 自己位置未知:不参与分派(下游按 pose 已知处理)
                p.stop()
            else:
                active.append(p)

        # 按 phase 对整队分派一次(角色分配等全队计算只在 _act_* 里算一次)。
        try:
            if phase == Phase.NORMAL:
                _act_normal(context, active, store)
            elif phase == Phase.OUR_KICKOFF:
                _clear_normal_sticky(store)
                _act_our_kickoff(context, active, store)
            elif phase == Phase.OPP_KICKOFF:
                _clear_normal_sticky(store)
                _act_opp_kickoff(context, active)
            elif phase == Phase.OUR_SET_PLAY:
                _clear_normal_sticky(store)
                _act_our_set_play(context, active, store)
            elif phase == Phase.OPP_SET_PLAY:
                _clear_normal_sticky(store)
                _act_opp_set_play(context, active, store)
            elif phase == Phase.READY:
                _clear_normal_sticky(store)
                _act_ready(context, active)
            elif phase == Phase.STOPPED:
                _clear_normal_sticky(store)
                for p in active:
                    p.action = "stopped"
                    p.stop()
        except Exception:
            _log.exception("Phase %s crashed, stopping all players", phase.value)
            for p in active:
                p.stop()

        # 更新球历史（用于下一帧的速度预测）
        if context.ball is not None:
            store.prev_ball_x = context.ball.x
            store.prev_ball_y = context.ball.y
            store.prev_ball_time = context.ball.last_seen_at

        # 队员可视化统一在最后画一遍:覆盖所有球员(含判罚/未就绪/STOPPED),
        # 修复 SET 等状态下红球/标签消失的问题。
        for p in players:
            _draw_teammate_marker(p)


def _should_keep_away(context: Context, store) -> bool:
    """判断是否应进入 keep-away 控球拖延模式。"""
    g = context.game
    if g is None:
        return False
    if g.secs_remaining <= 0 or g.secs_remaining > KEEP_AWAY_TIME:
        return False
    if len(g.teams) < 2:
        return False
    our_team = g.teams[0] if g.teams[0].team_number == context.team_id else g.teams[1]
    opp_team = g.teams[1] if our_team is g.teams[0] else g.teams[0]
    if our_team.score < opp_team.score + KEEP_AWAY_GOAL_DIFF:
        return False
    return True


def _act_keep_away(context: Context, players: list[Player], store) -> None:
    """Keep-away:领先+末段,控球拖延时间。

    带球者走向己方角落;被逼抢则传最远的队友;无球队员拉开接应。
    """
    if not players:
        return

    ball = context.ball
    attacker = _select_closest_attacker(context, players, store=store)
    rest = [p for p in players if p is not attacker]
    gx, gy = own_goal(context)
    guard = min(rest, key=lambda p: dist(p.pose.x, p.pose.y, gx, gy)) if rest else None
    supporters = [p for p in rest if p is not guard] if guard else rest

    if guard is not None:
        guard.guard()

    attacker.action = "keep_away"
    attacker.keep_away()

    for s in supporters:
        s.action = "keep_away_support"
        if ball is not None and s.pose is not None:
            # 走到球的另一侧,拉开空间
            spread_x = ball.x - 2.0 if ball.x > 0 else ball.x + 2.0
            spread_y = ball.y + (2.0 if s.id % 2 == 0 else -2.0)
            half_l = context.field.length / 2.0 - 0.3
            half_w = context.field.width / 2.0 - 0.3
            spread_x = clamp(spread_x, -half_l, half_l)
            spread_y = clamp(spread_y, -half_w, half_w)
            s.walk_to(
                (spread_x, spread_y),
                avoid_ball=True, avoid_robots=True,
            )


def _clear_normal_sticky(store) -> None:
    store.normal_attacker = None


def _player_dist_to_ball(context: Context, p: Player) -> float:
    """球员到球当前位置的距离。"""
    ball = context.ball
    return (
        dist(p.pose.x, p.pose.y, ball.x, ball.y) + _fallen_time_cost(p)
        if ball is not None else math.inf
    )


def _fallen_time_cost(p: Player) -> float:
    return FALLEN_COST if p.is_fallen else 0.0


def _predict_ball_pos(
    context: Context, store,
    predict_ahead: float = 0.5,
) -> tuple[float, float] | None:
    """根据历史位置预测球在 ``predict_ahead`` 秒后的坐标。"""
    ball = context.ball
    if ball is None:
        return None

    prev_x = getattr(store, "prev_ball_x", None)
    prev_y = getattr(store, "prev_ball_y", None)
    prev_t = getattr(store, "prev_ball_time", None)

    if prev_x is not None and prev_y is not None and prev_t is not None:
        dt = ball.last_seen_at - prev_t
        if 0.001 < dt < 1.0:
            vx = (ball.x - prev_x) / dt
            vy = (ball.y - prev_y) / dt
            return (ball.x + vx * predict_ahead, ball.y + vy * predict_ahead)

    return (ball.x, ball.y)


def _select_closest_attacker(
    context: Context,
    players: list[Player],
    preferred_id: int | None = None,
    store=None,
) -> Player:
    """选到预测球位距离最小的球员。"""
    # 使用预测球位
    if store is not None:
        pred = _predict_ball_pos(context, store)
    else:
        pred = (context.ball.x, context.ball.y) if context.ball is not None else (0.0, 0.0)
    target_x, target_y = pred if pred is not None else (0.0, 0.0)

    ranked = [
        (p, dist(p.pose.x, p.pose.y, target_x, target_y) + _fallen_time_cost(p))
        for p in players
    ]
    best, best_dist = min(ranked, key=lambda item: item[1])
    preferred = next((item for item in ranked if item[0].id == preferred_id), None)
    if (
        preferred is not None
        and preferred[1] <= best_dist + ATTACKER_KEEP_DIST_MARGIN_M
    ):
        return preferred[0]
    return best


def _most_threatening_opponent(context: Context) -> int | None:
    """返回距我方球门最近的对方球员 id（最危险）。"""
    gx, gy = own_goal(context)
    best_id = None
    best_dist = math.inf
    for oid, opp in context.opponents.items():
        if opp.pose is None:
            continue
        d = dist(opp.pose.x, opp.pose.y, gx, gy)
        if d < best_dist:
            best_dist = d
            best_id = oid
    return best_id


def _act_normal(context: Context, players: list[Player], store) -> None:
    """NORMAL:根据球位置切换攻防阵型,含盯人防守和停滞换人。"""
    if not players:
        return

    # P2-14: 保持球权拖延
    if _should_keep_away(context, store):
        _act_keep_away(context, players, store)
        return

    ball = context.ball
    if ball is None:
        for p in players:
            p.stop()
        return

    # --- 停滞检测(跨越所有模式) ---
    attacker = _select_closest_attacker(
        context, players, getattr(store, "normal_attacker", None), store,
    )
    attacker_dist = dist(attacker.pose.x, attacker.pose.y, ball.x, ball.y) + (
        _fallen_time_cost(attacker)
    )

    prev_dist = getattr(store, "attacker_prev_dist", None)
    if prev_dist is not None and attacker.id == getattr(store, "normal_attacker", None):
        dist_delta = prev_dist - attacker_dist
        if dist_delta < ATTACKER_STUCK_DIST_DELTA:
            store.attacker_stuck_counter = getattr(store, "attacker_stuck_counter", 0) + 1
        else:
            store.attacker_stuck_counter = 0

        if store.attacker_stuck_counter >= ATTACKER_STUCK_FRAMES_MAX:
            store.normal_attacker = None
            store.attacker_stuck_counter = 0
            attacker = _select_closest_attacker(context, players, None, store)
            attacker_dist = dist(attacker.pose.x, attacker.pose.y, ball.x, ball.y) + (
                _fallen_time_cost(attacker)
            )
    else:
        store.attacker_stuck_counter = 0

    store.normal_attacker = attacker.id
    store.attacker_prev_dist = attacker_dist

    # --- 攻防权重: 根据球位决定模式 ---
    defense_mode = ball.x < DEFEND_BALL_X
    attack_mode = ball.x > ATTACK_BALL_X

    if attack_mode:
        # 进攻模式: 双前锋前压(至少3人活跃才有secondary)
        attacker.action = "attack"
        attacker.attack()
        rest = [p for p in players if p is not attacker]
        if len(rest) >= 2:
            secondary = min(
                rest,
                key=lambda p: dist(p.pose.x, p.pose.y, ball.x, ball.y) + _fallen_time_cost(p),
            )
            secondary.action = "attack"
            secondary.attack()
            rest = [p for p in rest if p is not secondary]
        if rest:
            gx, gy = own_goal(context)
            guard = min(rest, key=lambda p: dist(p.pose.x, p.pose.y, gx, gy))
            guard.guard()
        return

    if defense_mode:
        # 防守模式: 一人逼抢,两人密集防守(含盯人)
        attacker.action = "attack"
        attacker.attack()
        rest = [p for p in players if p is not attacker]
        if rest:
            gx, gy = own_goal(context)
            guard = min(rest, key=lambda p: dist(p.pose.x, p.pose.y, gx, gy))
            guard.guard()
            rest = [p for p in rest if p is not guard]
        threat_id = _most_threatening_opponent(context)
        for p in rest:
            p.action = "support"
            p.support(mark_opponent_id=threat_id)
        return

    # 均衡模式: 当前默认 (attacker + guard + support)
    attacker.action = "attack"
    attacker.attack()
    rest = [p for p in players if p is not attacker]
    if rest:
        gx, gy = own_goal(context)
        guard = min(rest, key=lambda p: dist(p.pose.x, p.pose.y, gx, gy))
        guard.guard()
        rest = [p for p in rest if p is not guard]
    threat_id = _most_threatening_opponent(context)
    for p in rest:
        p.action = "support"
        p.support(mark_opponent_id=threat_id)


def _act_our_kickoff(context: Context, players: list[Player], store) -> None:
    """OUR_KICKOFF:短传给最近的队友,接应队员前插接球,守门员不变。"""
    if not players:
        return

    active_ids = {p.id for p in players}
    if store.prev_phase != Phase.OUR_KICKOFF or store.kickoff_taker not in active_ids:
        store.kickoff_taker = _select_closest_attacker(context, players, store=store).id

    attacker_id = store.kickoff_taker
    attacker = next((p for p in players if p.id == attacker_id), None)
    if attacker is None:
        return

    rest = [p for p in players if p is not attacker]
    gx, gy = own_goal(context)
    guard = min(rest, key=lambda p: dist(p.pose.x, p.pose.y, gx, gy)) if rest else None
    supporters = [p for p in rest if p is not guard] if guard else rest

    if guard is not None:
        guard.guard()

    ball = context.ball

    if store.prev_phase != Phase.OUR_KICKOFF:
        if supporters and ball is not None:
            target = min(supporters, key=lambda p: dist(
                p.pose.x, p.pose.y, ball.x, ball.y,
            ))
            if target.pose is not None:
                kick_dir = angle_to(ball.x, ball.y, target.pose.x, target.pose.y)
                attacker.action = "kickoff_pass"
                attacker.kick(kick_dir, power=2.5)
            else:
                attacker.action = "kickoff"
                attacker.kick(0.1, KICK_POWER_OUR_KICKOFF)
        else:
            attacker.action = "kickoff"
            attacker.kick(0.1, KICK_POWER_OUR_KICKOFF)

        for p in supporters:
            p.action = "kickoff_support"
            if ball is not None:
                p.walk_to(
                    (ball.x + 2.0, p.pose.y * 0.5),
                    avoid_ball=True, avoid_robots=True,
                )
    else:
        _act_normal(context, players, store)


def _act_opp_kickoff(context: Context, players: list[Player]) -> None:
    """对方开球:一人守门,其余人站到中圈外固定点等待。"""
    if not players:
        return
    gx, gy = own_goal(context)
    guard = min(players, key=lambda p: dist(p.pose.x, p.pose.y, gx, gy))
    guard.guard()

    rest = [p for p in players if p is not guard]
    r = context.field.circle_radius
    slots = [(-r - 0.5, 0.0), (-r - 2.0, 0.5)]
    for p, target in zip(rest, slots):
        p.action = "opp_kickoff:ready"
        p.walk_to(target, avoid_ball=True, avoid_robots=True)


def _act_our_set_play(context: Context, players: list[Player], store) -> None:
    """OUR_SET_PLAY:按定位球类型分派,首帧执行对应战术,后续按正常拼抢。"""
    set_play = get_set_play_type(context)
    if not players:
        return

    is_first = (store.prev_phase != Phase.OUR_SET_PLAY)

    attacker = _select_closest_attacker(context, players, store=store)
    rest = [p for p in players if p is not attacker]
    gx, gy = own_goal(context)
    guard = min(rest, key=lambda p: dist(p.pose.x, p.pose.y, gx, gy)) if rest else None
    supporters = [p for p in rest if p is not guard] if guard else rest

    if guard is not None:
        guard.guard()

    ball = context.ball

    if is_first and ball is not None:
        if set_play == SetPlay.CORNER_KICK:
            _exec_corner_kick(context, attacker, supporters, ball)
        elif set_play == SetPlay.GOAL_KICK:
            _exec_goal_kick(attacker, supporters, ball)
        elif set_play == SetPlay.THROW_IN:
            _exec_throw_in(attacker, supporters, ball)
        elif set_play == SetPlay.DIRECT_FREE_KICK:
            _exec_direct_free_kick(attacker, supporters, ball)
        elif set_play == SetPlay.INDIRECT_FREE_KICK:
            _exec_indirect_free_kick(attacker, supporters, ball)
        elif set_play == SetPlay.PENALTY_KICK:
            _exec_penalty_kick(attacker, supporters, ball)
        else:
            _act_normal(context, players, store)
    else:
        _act_normal(context, players, store)


def _exec_corner_kick(context, attacker, supporters, ball):
    """角球:直接旋向球门,支援球员前插远门柱。"""
    goal_center = opponent_goal(context)
    kick_dir = math.atan2(goal_center[1] - ball.y, goal_center[0] - ball.x)
    attacker.action = "corner_kick"
    attacker.kick(kick_dir, SET_PLAY_POWER_CORNER)
    for s in supporters:
        s.action = "corner_support"
        if s.pose is not None:
            s.walk_to(
                (ball.x + 2.0, s.pose.y * 0.5),
                avoid_ball=True, avoid_robots=True,
            )


def _exec_goal_kick(attacker, supporters, ball):
    """球门球:开大脚到中场附近,接应队员前压。"""
    if supporters:
        target = max(
            supporters, key=lambda p: p.pose.x if p.pose is not None else -999,
        )
        if target.pose is not None:
            kick_dir = angle_to(ball.x, ball.y, target.pose.x, target.pose.y)
        else:
            kick_dir = 0.0
        attacker.action = "goal_kick"
        attacker.kick(kick_dir, SET_PLAY_POWER_GOAL_KICK)
        for s in supporters:
            s.action = "goal_kick_support"
            s.walk_to((0.0, ball.y * 0.3), avoid_ball=True, avoid_robots=True)
    else:
        attacker.action = "goal_kick"
        attacker.kick(0.0, SET_PLAY_POWER_GOAL_KICK)


def _exec_throw_in(attacker, supporters, ball):
    """界外球:传给最近的队友。"""
    if supporters:
        target = min(
            supporters, key=lambda p: dist(
                p.pose.x, p.pose.y, ball.x, ball.y,
            ) if p.pose is not None else math.inf,
        )
        if target.pose is not None:
            kick_dir = angle_to(ball.x, ball.y, target.pose.x, target.pose.y)
        else:
            kick_dir = 0.0
        attacker.action = "throw_in"
        attacker.kick(kick_dir, SET_PLAY_POWER_THROW_IN)
        for s in supporters:
            s.action = "throw_in_support"
            if s.pose is not None:
                s.walk_to(
                    (ball.x + 2.0, s.pose.y * 0.5),
                    avoid_ball=True, avoid_robots=True,
                )
    else:
        attacker.action = "throw_in"
        attacker.kick(0.0, SET_PLAY_POWER_THROW_IN)


def _exec_direct_free_kick(attacker, supporters, ball):
    """直接任意球:尝试射门,支援球员前插抢点。"""
    kick_plan = attacker.plan_kick()
    if kick_plan is not None:
        kick_dir, power = kick_plan
        attacker.action = "free_kick"
        attacker.kick(kick_dir, power)
    else:
        attacker.action = "free_kick"
        attacker.kick(0.0, SET_PLAY_POWER_FREE_KICK)
    for s in supporters:
        s.action = "free_kick_support"
        if s.pose is not None:
            s.walk_to(
                (ball.x + 2.0, ball.y * 0.3),
                avoid_ball=True, avoid_robots=True,
            )


def _exec_indirect_free_kick(attacker, supporters, ball):
    """间接任意球:短传队友,接球人立刻射门/进攻。"""
    if supporters:
        target = min(
            supporters, key=lambda p: dist(
                p.pose.x, p.pose.y, ball.x, ball.y,
            ) if p.pose is not None else math.inf,
        )
        if target.pose is not None:
            kick_dir = angle_to(ball.x, ball.y, target.pose.x, target.pose.y)
            attacker.action = "indirect_free_kick"
            attacker.kick(kick_dir, SET_PLAY_POWER_INDIRECT)
            for s in supporters:
                s.action = "indirect_support"
                if s is target:
                    s.attack()
                elif s.pose is not None:
                    s.walk_to(
                        (ball.x + 1.5, ball.y),
                        avoid_ball=True, avoid_robots=True,
                    )
        else:
            attacker.action = "indirect_free_kick"
            attacker.kick(0.0, SET_PLAY_POWER_INDIRECT)
    else:
        attacker.action = "indirect_free_kick"
        attacker.kick(0.0, SET_PLAY_POWER_INDIRECT)


def _exec_penalty_kick(attacker, supporters, ball):
    """点球:大力射向球门正中,其他人等在禁区外。"""
    goal_center = opponent_goal(attacker.context)
    kick_dir = angle_to(ball.x, ball.y, goal_center[0], goal_center[1])
    attacker.action = "penalty_kick"
    attacker.kick(kick_dir, SET_PLAY_POWER_PENALTY)
    for s in supporters:
        s.action = "penalty_support"
        if s.pose is not None:
            s.walk_to(
                (-1.0, s.pose.y * 0.3),
                avoid_ball=True, avoid_robots=True,
            )


def _act_opp_set_play(context: Context, players: list[Player], store) -> None:
    """OPP_SET_PLAY:对方定位球,全员回防。

    守门员守门,其余球员在我方球门和球之间形成防线。
    """
    if not players:
        return

    gx, gy = own_goal(context)
    guard = min(players, key=lambda p: dist(p.pose.x, p.pose.y, gx, gy))
    guard.guard()

    rest = [p for p in players if p is not guard]
    if not rest:
        return

    ball = context.ball
    if ball is None:
        for p in rest:
            p.action = "opp_set_play:retreat"
            p.move_to_position((gx - 1.0, 0.0))
        return

    bx, by = ball.x, ball.y
    dx, dy = gx - bx, gy - by
    d = math.hypot(dx, dy)
    if d < 1e-6:
        ux, uy = -1.0, 0.0
    else:
        ux, uy = dx / d, dy / d

    half_l = context.field.length / 2.0
    half_w = context.field.width / 2.0 - 0.3

    for i, p in enumerate(rest):
        ratio = OPP_SET_PLAY_DEFEND_RATIO_BASE + 0.3 * i / max(len(rest) - 1, 1)
        along = min(ratio * d, d - 0.5)
        base_x = bx + ux * along
        base_y = by + uy * along
        spread = (i - (len(rest) - 1) / 2.0) * OPP_SET_PLAY_DEFEND_SPREAD
        tx = clamp(base_x + 0.0, -half_l + 0.3, half_l)
        ty = clamp(base_y + spread, -half_w, half_w)
        p.action = "opp_set_play:defend"
        p.move_to_position((tx, ty))


def _act_ready(context: Context, players: list[Player]) -> None:
    """READY:按角色分配站位,不再按 players 列表顺序。"""
    if not players:
        return

    game = context.game
    our_kickoff = game is not None and game.kicking_team == context.team_id
    field = context.field

    # 遴选守门员:距己方球门最近者
    gx, gy = own_goal(context)
    guard = min(players, key=lambda p: dist(p.pose.x, p.pose.y, gx, gy))
    rest = [p for p in players if p is not guard]

    if our_kickoff:
        # 守门员 → 小禁区线,准备接回传
        guard.action = "ready:guard"
        guard.walk_to(
            (-field.length / 2.0 + field.goal_area_length, 0.0),
            face=0.0,
            avoid_ball=True,
            avoid_robots=True,
        )
        if rest:
            # 主罚手 → 中圈旁
            kicker = min(
                rest, key=lambda p: dist(p.pose.x, p.pose.y, -field.circle_radius, 0.0),
            )
            kicker.action = "ready:kicker"
            kicker.walk_to(
                (-field.circle_radius, 0.0),
                face=0.0,
                avoid_ball=True,
                avoid_robots=True,
            )
            rest = [p for p in rest if p is not kicker]
        # 支援手 → 中场边路
        for p in rest:
            p.action = "ready:support"
            p.walk_to(
                (-0.5, field.circle_radius + 1.5),
                face=0.0,
                avoid_ball=True,
                avoid_robots=True,
            )
    else:
        # 守门员 → 门线
        guard.action = "ready:guard"
        guard.walk_to(
            (gx, 0.0),
            face=0.0,
            avoid_ball=True,
            avoid_robots=True,
        )
        if rest:
            # 拦截手(离中圈最近者) → 中圈外堵截
            interceptor = min(
                rest,
                key=lambda p: dist(
                    p.pose.x, p.pose.y, -field.circle_radius - 0.5, 0.0,
                ),
            )
            interceptor.action = "ready:interceptor"
            interceptor.walk_to(
                (-field.circle_radius - 0.5, 0.0),
                face=0.0,
                avoid_ball=True,
                avoid_robots=True,
            )
            rest = [p for p in rest if p is not interceptor]
        # 后卫 → 大禁区线补防
        for p in rest:
            p.action = "ready:defender"
            p.walk_to(
                (-field.length / 2.0 + field.penalty_area_length, 0.0),
                face=0.0,
                avoid_ball=True,
                avoid_robots=True,
            )



# ======================================================================
# 战场可视化 —— 显示球位置 + 球员到球的距离,画到 ROS 可视化
# ======================================================================

def _draw_teammate_marker(p: Player) -> None:
    """我方队员可视化:红色。踢球中→方块,否则→球体。

    每帧对所有球员统一调用(不受 phase/判罚/就绪影响)。标签两行:
    - 上:编号 + 当前高层动作(``p.action``),踢球中追加 ``[KICK]``。
    - 通过形状(方块 vs 球体)再次区分是否进入 kick 状态。
    """
    from .framework import debugdraw

    if p.pose is None:
        return
    red = (1.0, 0.2, 0.2)
    if p.is_kicking:
        debugdraw.cube(p.pose.x, p.pose.y, rgb=red, scale=0.38, ns="teammate")
    else:
        debugdraw.point(p.pose.x, p.pose.y, rgb=red, scale=0.3, ns="teammate")
    kick_tag = " [KICK]" if p.is_kicking else ""
    label = f"{p.id}:{p.action}{kick_tag}"
    debugdraw.text(p.pose.x, p.pose.y, label, rgb=(1.0, 0.9, 0.6), ns="teammate_id")


def _analyze_and_draw(context: Context, players: list[Player], store) -> None:
    """每帧:计算球员到球的距离,画可视化。

    不再依赖 analysis 模块;距离改为基于球当前位置。
    """
    from .framework import debugdraw

    ball = context.ball

    # 球不可见:无可视化
    if ball is None:
        return

    # 1. 画球当前位置(绿色点)
    debugdraw.point(ball.x, ball.y, rgb=(0.0, 1.0, 0.0), scale=0.2, ns="ball_current")

    # 2. 球员到球的距离:我方(红标签)+ 敌方(蓝标签)
    for p in players:
        if p.pose is None:
            continue
        d = dist(p.pose.x, p.pose.y, ball.x, ball.y)
        debugdraw.text(
            p.pose.x + 0.3, p.pose.y - 0.3, f"{d:.1f}m",
            rgb=(1.0, 0.6, 0.6), ns="dist_ours",
        )
    for r in context.opponents.values():
        if r.pose is None:
            continue
        d = dist(r.pose.x, r.pose.y, ball.x, ball.y)
        debugdraw.text(
            r.pose.x + 0.3, r.pose.y - 0.3, f"{d:.1f}m",
            rgb=(0.6, 0.6, 1.0), ns="dist_opp",
        )

