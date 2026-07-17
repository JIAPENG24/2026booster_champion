"""Default dynamic role implementations for SoccerSim.

Each role extends :class:`RoleStrategy` and assembles its subtree through
:meth:`build_subtree`. Methods such as ``target``, ``wants_to_kick``, and
``kick_target`` are implementation helpers for role utility nodes, not
base-class contracts.
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

import py_trees

from ..soccer_framework import BallState, PlayContext, Pose2D, ReadySlot
from ..tactics.geometry import normalize_angle
from ..tactics.targeting.ball_prediction import BallPredictor
from .nodes import AttackSubtreeConfig, MoveToTarget, build_attack_subtree
from .role import RoleStrategy

if TYPE_CHECKING:
    from ..runtime import SoccerKit


# ----------------------------------------------------------------------
# Chaser: approach and kick the ball
# ----------------------------------------------------------------------

# Approach alignment distance behind the ball while chasing, in meters.
_CHASER_APPROACH_OFFSET = 0.15


class ChaserRole(RoleStrategy):
    """Default chaser; kick target is split by ReadySlot into center shot or side clearance."""

    name = "chaser"

    _STATUS_LOG_INTERVAL_SEC = 2.0  # 0.5 Hz periodic status

    def __init__(self) -> None:
        self._last_decision: str | None = None
        self._last_status_log_at: float = 0.0
        self._lane_ema: float = 1.0  # EMA-smoothed lane score (1.0 = fully clear).

    def target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        ball = context.known_ball
        kt = self.kick_target(kit, player_id, context)
        kick_theta = math.atan2(kt.y - ball.y, kt.x - ball.x)
        approach = kit.motion.approach_target(
            ball,
            kick_theta,
            _CHASER_APPROACH_OFFSET,
        )
        self._log_status(kit, player_id, context, approach, kt)
        return approach

    def _log_status(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
        approach_target: Pose2D,
        kick_target: Pose2D,
    ) -> None:
        """Periodic (0.5 Hz) chaser status log for strategy evaluation."""
        logger = kit.logger
        if logger is None:
            return
        robot = context.teammates.get(player_id)
        if robot is None or robot.pose is None:
            return
        now = time.monotonic()
        if now - self._last_status_log_at < self._STATUS_LOG_INTERVAL_SEC:
            return
        self._last_status_log_at = now
        ball = context.known_ball
        pose = robot.pose
        shot_zone = kit.targeting.shot_zone_allowed(ball)
        lane_score = kit.targeting.shot_lane_score(context)
        was_dribbling = self._last_decision == "dribble"
        logger.info(
            f"chaser status player={player_id} decision={self._last_decision} "
            f"pose=({pose.x:.2f},{pose.y:.2f}) "
            f"target=({approach_target.x:.2f},{approach_target.y:.2f}) "
            f"kick_target=({kick_target.x:.2f},{kick_target.y:.2f}) "
            f"ball=({ball.x:.2f},{ball.y:.2f}) "
            f"shot_zone={shot_zone} lane={lane_score:.2f} dribbling={was_dribbling}",
            event="chaser_status",
            player_id=player_id,
            decision=self._last_decision,
            pose_x=round(pose.x, 2),
            pose_y=round(pose.y, 2),
            target_x=round(approach_target.x, 2),
            target_y=round(approach_target.y, 2),
            kick_target_x=round(kick_target.x, 2),
            kick_target_y=round(kick_target.y, 2),
            ball_x=round(ball.x, 2),
            ball_y=round(ball.y, 2),
            shot_zone_allowed=shot_zone,
            lane_score=round(lane_score, 2),
            was_dribbling=was_dribbling,
        )

    def kick_target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        # EMA smoothing: blend raw lane score to suppress per-frame obstacle jitter.
        raw_lane = kit.targeting.shot_lane_score(context)
        self._lane_ema = 0.3 * raw_lane + 0.7 * self._lane_ema

        slot = kit.config.ready_slot_for_player(player_id)
        target, decision = kit.targeting.select_kick_target(
            player_id,
            context,
            kit.is_player_allowed,
            was_shooting=self._last_decision == "shoot",
            was_dribbling=self._last_decision == "dribble",
            lane_ema=self._lane_ema,
        )

        if decision != self._last_decision:
            self._last_decision = decision
            logger = kit.logger
            if logger is not None:
                extra: dict[str, object] = {}
                msg = (
                    f"chaser kick decision={decision} "
                    f"ball=({context.known_ball.x:.3f},{context.known_ball.y:.3f}) "
                    f"target=({target.x:.3f},{target.y:.3f}) "
                    f"player={player_id} slot={slot.value}"
                )
                if decision in ("shoot", "dribble"):
                    lane_score = kit.targeting.shot_lane_score(context)
                    extra["shot_lane_score"] = round(lane_score, 3)
                    msg += f" lane_score={lane_score:.3f}"
                logger.info(
                    msg,
                    event="chaser_kick_decision",
                    player_id=player_id,
                    slot=slot.value,
                    decision=decision,
                    ball_x=round(context.known_ball.x, 3),
                    ball_y=round(context.known_ball.y, 3),
                    target_x=round(target.x, 3),
                    target_y=round(target.y, 3),
                    **extra,
                )

                ball = context.known_ball
                lane_score = kit.targeting.shot_lane_score(context)
                if decision == "shoot":
                    logger.info(
                        f"shot attempt ball=({ball.x:.3f},{ball.y:.3f}) "
                        f"lane={lane_score:.3f} player={player_id}",
                        event="shot_attempted",
                        console=False,
                        player_id=player_id,
                        slot=slot.value,
                        ball_x=round(ball.x, 3),
                        ball_y=round(ball.y, 3),
                        lane_score=round(lane_score, 3),
                    )
                elif decision == "pass":
                    logger.info(
                        f"pass attempt from=({ball.x:.3f},{ball.y:.3f}) "
                        f"to=({target.x:.3f},{target.y:.3f}) player={player_id}",
                        event="pass_attempted",
                        console=False,
                        player_id=player_id,
                        slot=slot.value,
                        ball_x=round(ball.x, 3),
                        ball_y=round(ball.y, 3),
                        target_x=round(target.x, 3),
                        target_y=round(target.y, 3),
                    )

        return target

    def _approach_reason(self, kit: "SoccerKit", player_id: int) -> str:
        slot = kit.config.ready_slot_for_player(player_id)
        return f"{slot.value} approach ball"

    def _kick_reason(
        self,
        kit: "SoccerKit",
        player_id: int,
        target: Pose2D,
    ) -> str:
        slot = kit.config.ready_slot_for_player(player_id)
        decision = self._last_decision or "unknown"
        default = f"chaser kick decision={decision}"
        return kit.targeting.kick_reason(target, default=default)

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return build_attack_subtree(
            kit,
            player_id,
            AttackSubtreeConfig(
                target_fn=lambda context: self.target(kit, player_id, context),
                kick_target_fn=lambda context: self.kick_target(
                    kit,
                    player_id,
                    context,
                ),
                reason_fn=lambda: self._approach_reason(kit, player_id),
                kick_reason_fn=lambda target: self._kick_reason(
                    kit,
                    player_id,
                    target,
                ),
                speed_multiplier=2.0,
                kick_power=2.5,
            ),
        )


# ----------------------------------------------------------------------
# Supporter: attacking support position
# ----------------------------------------------------------------------


class SupporterRole(RoleStrategy):
    """Attacking support role; uses :meth:`Targeting.support_target` for positioning."""

    name = "supporter"

    _PUSHOUT_LOG_INTERVAL_SEC = 2.0
    _STATUS_LOG_INTERVAL_SEC = 2.0  # 0.5 Hz periodic status
    _SMOOTH_SNAP_DISTANCE_M = 1.5  # Beyond this, snap instead of smoothing (chaser/role switch).

    def __init__(self) -> None:
        self._was_pushed: bool = False
        self._pushout_logged: bool = False
        self._last_pushout_log_at: float = 0.0
        self._last_decision: str | None = None
        self._last_status_log_at: float = 0.0
        self._last_mode: str | None = None
        self._was_willing_to_kick: bool = False
        # Anti-jitter target smoothing (rate-limited toward the raw support target).
        self._smoothed_target: Pose2D | None = None
        self._last_smooth_t: float | None = None
        # Danger-zone state tracking for event logging.
        self._danger_zone_active: bool = False

    def target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        # Anchor to the ASSIGNED chaser (read from the cached RoleAssignment)
        # rather than letting support_target re-derive it by scanning teammates.
        # When the goalkeeper is the assigned chaser (ball in the defensive area
        # and keeper closest — chaser_id == goalkeeper_id), no outfield player has
        # the chaser role; the supporter switches to fixed-field-reference
        # danger-zone positioning so the two outfielders no longer anchor to each
        # other and drift to the corner flag.
        chaser_id: int | None = None
        danger_zone = False
        roles = kit.current_roles
        if roles is not None:
            cid = roles.chaser_id
            gkid = roles.goalkeeper_id
            if cid is not None and gkid is not None and cid == gkid:
                danger_zone = True
            elif cid is not None:
                chaser_id = cid

        if danger_zone != self._danger_zone_active:
            self._danger_zone_active = danger_zone
            logger = kit.logger
            if logger is not None:
                gk_id = kit.config.goalkeeper_player_id()
                outfield_ids = sorted(kit.config.player_ids)
                if gk_id in outfield_ids:
                    outfield_ids.remove(gk_id)
                try:
                    idx = outfield_ids.index(player_id)
                except ValueError:
                    idx = player_id
                dz_role = "cover" if idx % 2 == 0 else "outlet"
                logger.info(
                    f"supporter danger_zone={danger_zone} player={player_id} role={dz_role} "
                    f"ball=({context.known_ball.x:.3f},{context.known_ball.y:.3f})",
                    event="supporter_danger_zone",
                    player_id=player_id,
                    active=danger_zone,
                    role=dz_role,
                    ball_x=round(context.known_ball.x, 3),
                    ball_y=round(context.known_ball.y, 3),
                )

        raw, was_pushed = kit.targeting.support_target(
            player_id,
            context,
            kit.is_player_allowed,
            chaser_id=chaser_id,
            danger_zone=danger_zone,
        )
        target = self._smooth_target(kit, raw)

        if was_pushed != self._was_pushed:
            self._was_pushed = was_pushed
            if was_pushed:
                self._pushout_logged = False
            logger = kit.logger
            if logger is not None and was_pushed and not self._pushout_logged:
                now = time.monotonic()
                if now - self._last_pushout_log_at >= self._PUSHOUT_LOG_INTERVAL_SEC:
                    self._last_pushout_log_at = now
                    self._pushout_logged = True
                    logger.info(
                        f"supporter pushout player={player_id} "
                        f"target=({target.x:.3f},{target.y:.3f}) "
                        f"ball=({context.known_ball.x:.3f},{context.known_ball.y:.3f})",
                        event="supporter_pushout",
                        player_id=player_id,
                        target_x=round(target.x, 3),
                        target_y=round(target.y, 3),
                        ball_x=round(context.known_ball.x, 3),
                        ball_y=round(context.known_ball.y, 3),
                    )

        self._log_status(kit, player_id, context, target)
        return target

    def _smooth_target(self, kit: "SoccerKit", raw: Pose2D) -> Pose2D:
        """Rate-limit the supporter target toward ``raw`` to absorb frame jitter.

        Mirrors the GK ``gk_target_smooth_speed`` pattern: the smoothed position
        chases ``raw`` at most ``support_target_smooth_speed`` m/s. A large jump
        (> snap distance) snaps immediately so chaser/role switches are not
        artificially lagged. ``theta`` is taken straight from ``raw`` (heading
        tracks the ball without smoothing).
        """

        smooth_speed = kit.config.strategy.support_target_smooth_speed
        now = time.monotonic()
        prev = self._smoothed_target
        if prev is None or smooth_speed <= 0.0:
            self._smoothed_target = Pose2D(raw.x, raw.y, raw.theta)
            self._last_smooth_t = now
            return self._smoothed_target

        dt = 0.0
        if self._last_smooth_t is not None:
            dt = max(0.0, now - self._last_smooth_t)
        if dt <= 0.0:
            # Same clock tick: hold smoothed position, adopt raw heading.
            self._smoothed_target = Pose2D(prev.x, prev.y, raw.theta)
            return self._smoothed_target

        dx = raw.x - prev.x
        dy = raw.y - prev.y
        dist = math.hypot(dx, dy)
        if dist > self._SMOOTH_SNAP_DISTANCE_M:
            nx, ny = raw.x, raw.y
        else:
            step = min(dist, smooth_speed * dt)
            nx = prev.x + dx * (step / dist) if dist > 1e-6 else prev.x
            ny = prev.y + dy * (step / dist) if dist > 1e-6 else prev.y

        self._smoothed_target = Pose2D(nx, ny, raw.theta)
        self._last_smooth_t = now
        return self._smoothed_target

    def _log_status(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
        target: Pose2D,
    ) -> None:
        """Periodic (0.5 Hz) or event-triggered supporter status log.

        Tracks chaser-relative distance, triangle angle, facing error, and
        positioning mode (backup / closein / hold / fallback) for evaluating
        the supporter strategy.
        """

        logger = kit.logger
        if logger is None:
            return

        ball = context.known_ball
        own_robot = context.teammates.get(player_id)
        if own_robot is None or own_robot.pose is None:
            return

        own_pose = own_robot.pose
        sb_dist = math.hypot(ball.x - own_pose.x, ball.y - own_pose.y)

        # Find chaser (non-GK teammate closest to ball)
        gk_id = kit.config.goalkeeper_player_id()
        game = context.known_game
        chaser_id: int | None = None
        chaser_pose: Pose2D | None = None
        chaser_ball_dist = float("inf")
        for tid, trobot in context.teammates.items():
            if (
                tid == player_id
                or trobot.pose is None
                or not kit.is_player_allowed(game, tid)
            ):
                continue
            if tid == gk_id:
                continue
            dist = math.hypot(ball.x - trobot.pose.x, ball.y - trobot.pose.y)
            if dist < chaser_ball_dist:
                chaser_ball_dist = dist
                chaser_id = tid
                chaser_pose = trobot.pose

        # Compute metrics and mode
        if chaser_pose is not None:
            sc_dist = math.hypot(
                own_pose.x - chaser_pose.x, own_pose.y - chaser_pose.y,
            )
            cb_dist = chaser_ball_dist
            if sc_dist < 1.0:
                mode = "backup"
            elif sc_dist > 2.8:
                mode = "closein"
            else:
                mode = "hold"

            # Triangle angle at chaser: angle between chaser→ball and chaser→supporter
            vcb_x = ball.x - chaser_pose.x
            vcb_y = ball.y - chaser_pose.y
            vcs_x = own_pose.x - chaser_pose.x
            vcs_y = own_pose.y - chaser_pose.y
            vcb_len = math.hypot(vcb_x, vcb_y)
            vcs_len = math.hypot(vcs_x, vcs_y)
            if vcb_len > 1e-6 and vcs_len > 1e-6:
                dot = max(-1.0, min(1.0,
                    (vcb_x * vcs_x + vcb_y * vcs_y) / (vcb_len * vcs_len),
                ))
                tri_angle = round(math.degrees(math.acos(dot)), 1)
            else:
                tri_angle = -1.0
        else:
            sc_dist = -1.0
            cb_dist = -1.0
            tri_angle = -1.0
            mode = "fallback"

        # Facing error: how well supporter theta points at ball
        desired_theta = math.atan2(ball.y - own_pose.y, ball.x - own_pose.x)
        facing_err = abs(normalize_angle(own_pose.theta - desired_theta))

        # Event trigger: mode change
        mode_changed = mode != self._last_mode
        if mode_changed:
            self._last_mode = mode

        # Throttle: 0.5 Hz periodic or event-triggered
        now = time.monotonic()
        if not mode_changed and now - self._last_status_log_at < self._STATUS_LOG_INTERVAL_SEC:
            return
        self._last_status_log_at = now

        chaser_pose_str = (
            f"chaser_pose=({chaser_pose.x:.2f},{chaser_pose.y:.2f}) "
            if chaser_pose is not None
            else ""
        )
        dz_role: str | None = None
        if self._danger_zone_active:
            outfield_ids = sorted(kit.config.player_ids)
            if gk_id in outfield_ids:
                outfield_ids.remove(gk_id)
            try:
                idx = outfield_ids.index(player_id)
                dz_role = "cover" if idx % 2 == 0 else "outlet"
            except ValueError:
                pass
        logger.info(
            f"supporter status player={player_id} mode={mode} "
            f"pose=({own_pose.x:.2f},{own_pose.y:.2f}) "
            f"sc_dist={sc_dist:.2f} sb_dist={sb_dist:.2f} cb_dist={cb_dist:.2f} "
            f"tri_angle={tri_angle:.1f} facing_err={facing_err:.2f} "
            f"chaser={chaser_id} {chaser_pose_str}"
            f"target=({target.x:.2f},{target.y:.2f}) "
            f"ball=({ball.x:.2f},{ball.y:.2f})"
            + (f" dz_role={dz_role}" if dz_role else ""),
            event="supporter_status",
            player_id=player_id,
            mode=mode,
            pose_x=round(own_pose.x, 2),
            pose_y=round(own_pose.y, 2),
            sc_dist=round(sc_dist, 2),
            sb_dist=round(sb_dist, 2),
            cb_dist=round(cb_dist, 2),
            triangle_angle=tri_angle,
            facing_error=round(facing_err, 2),
            facing_ball=facing_err < 0.3,
            chaser_id=chaser_id,
            chaser_pose_x=round(chaser_pose.x, 2) if chaser_pose is not None else None,
            chaser_pose_y=round(chaser_pose.y, 2) if chaser_pose is not None else None,
            target_x=round(target.x, 2),
            target_y=round(target.y, 2),
            ball_x=round(ball.x, 2),
            ball_y=round(ball.y, 2),
            danger_zone_role=dz_role,
        )

    def wants_to_kick(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> bool:
        ball = context.known_ball
        robot = context.teammates.get(player_id)
        if robot is None or robot.pose is None:
            self._was_willing_to_kick = False
            return False

        gk_id = kit.config.goalkeeper_player_id()
        game = context.known_game
        my_dist = math.hypot(ball.x - robot.pose.x, ball.y - robot.pose.y)

        min_gap = float("inf")
        for tid, trobot in context.teammates.items():
            if (
                tid == player_id
                or trobot.pose is None
                or not kit.is_player_allowed(game, tid)
            ):
                continue
            if tid == gk_id:
                continue
            gap = math.hypot(
                ball.x - trobot.pose.x, ball.y - trobot.pose.y,
            ) - my_dist
            if gap < min_gap:
                min_gap = gap

        # P5: asymmetric hysteresis — enter kick only when clearly alone,
        # exit only when teammate gets very close.
        if self._was_willing_to_kick:
            willing = min_gap > 0.5     # exit: teammate within 0.5 m
        else:
            willing = min_gap > 1.5     # enter: alone by 1.5 m

        self._was_willing_to_kick = willing
        return willing

    def kick_target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        target, decision = kit.targeting.select_kick_target(
            player_id,
            context,
            kit.is_player_allowed,
            was_shooting=self._last_decision == "shoot",
        )
        if decision != self._last_decision:
            self._last_decision = decision
            logger = kit.logger
            if logger is not None:
                extra: dict[str, object] = {}
                msg = (
                    f"supporter kick decision={decision} "
                    f"ball=({context.known_ball.x:.3f},{context.known_ball.y:.3f}) "
                    f"target=({target.x:.3f},{target.y:.3f}) "
                    f"player={player_id}"
                )
                if decision in ("shoot", "dribble"):
                    lane_score = kit.targeting.shot_lane_score(context)
                    extra["shot_lane_score"] = round(lane_score, 3)
                    msg += f" lane_score={lane_score:.3f}"
                logger.info(
                    msg,
                    event="supporter_kick_decision",
                    player_id=player_id,
                    decision=decision,
                    ball_x=round(context.known_ball.x, 3),
                    ball_y=round(context.known_ball.y, 3),
                    target_x=round(target.x, 3),
                    target_y=round(target.y, 3),
                    **extra,
                )

                ball = context.known_ball
                lane_score = kit.targeting.shot_lane_score(context)
                if decision == "shoot":
                    logger.info(
                        f"supporter shot attempt ball=({ball.x:.3f},{ball.y:.3f}) "
                        f"lane={lane_score:.3f} player={player_id}",
                        event="shot_attempted",
                        console=False,
                        player_id=player_id,
                        role="supporter",
                        ball_x=round(ball.x, 3),
                        ball_y=round(ball.y, 3),
                        lane_score=round(lane_score, 3),
                    )
                elif decision == "pass":
                    logger.info(
                        f"supporter pass attempt from=({ball.x:.3f},{ball.y:.3f}) "
                        f"to=({target.x:.3f},{target.y:.3f}) player={player_id}",
                        event="pass_attempted",
                        console=False,
                        player_id=player_id,
                        role="supporter",
                        ball_x=round(ball.x, 3),
                        ball_y=round(ball.y, 3),
                        target_x=round(target.x, 3),
                        target_y=round(target.y, 3),
                    )
        return target

    def _kick_reason(
        self,
        kit: "SoccerKit",
        target: Pose2D,
    ) -> str:
        decision = self._last_decision or "unknown"
        return kit.targeting.kick_reason(target, default=f"supporter kick decision={decision}")

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return build_attack_subtree(
            kit,
            player_id,
            AttackSubtreeConfig(
                target_fn=lambda context: self.target(kit, player_id, context),
                kick_target_fn=lambda context: self.kick_target(
                    kit, player_id, context,
                ),
                wants_kick_fn=lambda context: self.wants_to_kick(
                    kit, player_id, context,
                ),
                reason_fn=lambda: "supporter hold",
                kick_reason_fn=lambda target: self._kick_reason(kit, target),
                hold_vyaw=0.25,
                strafe=True,
                speed_multiplier=2.0,
                kick_power=2.5,
            ),
        )


# ----------------------------------------------------------------------
# Defender: extension defensive position, defaulting to supporter target
# ----------------------------------------------------------------------


class DefenderRole(RoleStrategy):
    """Extension defensive role; default implementation reuses the supporter target."""

    name = "defender"

    def __init__(self) -> None:
        super().__init__()
        self._fallback = SupporterRole()

    def target(
        self,
        kit: "SoccerKit",
        player_id: int,
        context: PlayContext,
    ) -> Pose2D:
        return self._fallback.target(kit, player_id, context)

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return MoveToTarget(
            kit,
            player_id,
            lambda context: self.target(kit, player_id, context),
            reason_fn=lambda: "defender hold",
            hold_vyaw=0.12,
        )


# ----------------------------------------------------------------------
# Goalkeeper: guard the goal and clear dangerous balls
# ----------------------------------------------------------------------


class GoalkeeperRole(RoleStrategy):
    """Goalkeeper guarding and defensive-area clearance.

    Three-state machine driven by ball trajectory prediction:

    - **GUARD** — default arc positioning; ball not directly threatening.
    - **RUSH_OUT** — ball predicted to stop inside the defensive area; the
      keeper rushes to the predicted rest point to intercept and clear.
    - **LATERAL_BLOCK** — ball predicted to cross the goal line within the
      posts; the keeper moves laterally along the guard-depth line to block.

    State transitions are debounced (``gk_state_confirm_frames`` to enter,
    ``gk_state_release_frames`` to exit RUSH_OUT).
    """

    name = "goalkeeper"

    # Approach alignment distance for goalkeeper challenges (m).
    _APPROACH_OFFSET = 0.18

    # State constants
    _GUARD = 0
    _RUSH_OUT = 1
    _LATERAL = 2

    _STATE_NAMES = ("GUARD", "RUSH_OUT", "LATERAL")

    def __init__(self):
        super().__init__()
        self._predictor: BallPredictor | None = None
        self._predictor_init = False
        # State machine
        self._gk_state = self._GUARD
        self._pending_state = self._GUARD
        self._state_confirm = 0
        self._last_update_time: float = -1.0
        self._last_game_state: object = None
        self._lateral_entry_time: float = 0.0
        # Logging
        self._state_logged = False
        self._last_kick_dir_index = -1
        # Trajectory smoothing
        self._smooth_x = 0.0
        self._smooth_y = 0.0
        self._smooth_init = False

    # ------------------------------------------------------------------
    # Per-tick update (called once, idempotent)
    # ------------------------------------------------------------------

    def _ensure_updated(self, kit: "SoccerKit", context: PlayContext) -> None:
        """Update predictor and evaluate state machine once per tick."""
        ball = context.known_ball

        # Reset predictor and state on game-state change (e.g. READY→PLAYING)
        game = context.known_game
        if game is not None and game.state != self._last_game_state:
            self._last_game_state = game.state
            if self._predictor is not None:
                self._predictor.reset()
            self._gk_state = self._GUARD
            self._pending_state = self._GUARD
            self._state_confirm = 0
            self._smooth_init = False

        if ball.last_seen_at == self._last_update_time:
            return
        self._last_update_time = ball.last_seen_at

        if not self._predictor_init:
            strat = kit.config.strategy
            self._predictor = BallPredictor(
                history_size=strat.ball_prediction_history_size,
                kp=strat.ball_prediction_kp,
                ki=strat.ball_prediction_ki,
                kd=strat.ball_prediction_kd,
                friction_init=strat.ball_prediction_friction,
                max_horizon_sec=strat.ball_prediction_max_horizon_sec,
            )
            self._predictor_init = True

        self._predictor.update(ball.x, ball.y, ball.last_seen_at)
        self._update_gk_state(kit, context, ball)

    def _update_gk_state(
        self,
        kit: "SoccerKit",
        context: PlayContext,
        ball: BallState,
    ) -> None:
        """Evaluate state transitions with debouncing."""
        strat = kit.config.strategy

        # Ball in opponent half → force GUARD
        if ball.x >= 0.0:
            self._transition(self._GUARD, kit, ball, reason="ball in opponent half")
            return

        if not self._predictor.has_history:
            return

        # --- Evaluate conditions ---
        area_x, area_y = kit.targeting.goalkeeper_defensive_area()

        # RUSH_OUT: ball predicted to stop inside defensive area (with margin
        # to prevent oscillation when rest_point is near the area boundary).
        # Use a larger exit margin when already in RUSH_OUT to add hysteresis.
        rest_x, rest_y = self._predictor.predict_rest_point()
        if self._gk_state == self._RUSH_OUT:
            rush_margin = strat.gk_rush_out_exit_margin_m
        else:
            rush_margin = strat.gk_rush_out_margin_m
        rush_cond = rest_x < area_x - rush_margin and abs(rest_y) <= area_y

        # LATERAL: ball predicted to cross goal line within posts
        goal_x = kit.field.own_goal_x()
        goal_hw = kit.config.goal_width / 2.0
        crossing = self._predictor.predict_goal_crossing(goal_x, goal_hw)
        lateral_cond = crossing is not None and crossing[2]

        # Priority: LATERAL > RUSH_OUT > GUARD
        if lateral_cond:
            desired = self._LATERAL
        elif rush_cond:
            desired = self._RUSH_OUT
        else:
            desired = self._GUARD

        # LATERAL hold: prevent oscillation by requiring a minimum dwell time
        # before leaving LATERAL state.
        if self._gk_state == self._LATERAL and desired != self._LATERAL:
            elapsed = ball.last_seen_at - self._lateral_entry_time
            if elapsed < strat.gk_lateral_hold_min_sec:
                return

        # --- Debounce ---
        if desired == self._gk_state:
            self._state_confirm = 0
            self._pending_state = self._gk_state
            return

        if desired != self._pending_state:
            self._pending_state = desired
            self._state_confirm = 1
        else:
            self._state_confirm += 1

        # Higher threshold to leave RUSH_OUT
        if self._gk_state == self._RUSH_OUT and desired != self._RUSH_OUT:
            threshold = strat.gk_state_release_frames
        else:
            threshold = strat.gk_state_confirm_frames

        if self._state_confirm >= threshold:
            self._transition(desired, kit, ball)

    def _transition(
        self,
        new_state: int,
        kit: "SoccerKit",
        ball: BallState,
        reason: str = "",
    ) -> None:
        """Execute a state transition and log it."""
        if new_state == self._gk_state:
            self._state_confirm = 0
            self._pending_state = new_state
            return

        old_name = self._STATE_NAMES[self._gk_state]
        new_name = self._STATE_NAMES[new_state]
        self._gk_state = new_state
        self._state_confirm = 0
        self._pending_state = new_state
        self._state_logged = False
        # Record entry timestamp for LATERAL hold time
        if new_state == self._LATERAL:
            self._lateral_entry_time = ball.last_seen_at
        # Reset kick direction lock on every transition
        self._last_kick_dir_index = -1

        logger = kit.logger
        if logger is not None:
            extra = ""
            if reason:
                extra = f" reason={reason}"
            logger.info(
                f"GK {old_name}→{new_name} "
                f"ball=({ball.x:.3f},{ball.y:.3f}){extra}",
                event="goalkeeper_state_transition",
                old_state=old_name,
                new_state=new_name,
                ball_x=round(ball.x, 3),
                ball_y=round(ball.y, 3),
                reason=reason,
            )

    # ------------------------------------------------------------------
    # Public RoleStrategy interface
    # ------------------------------------------------------------------

    def target(
        self,
        kit: "SoccerKit",
        context: PlayContext,
    ) -> Pose2D:
        self._ensure_updated(kit, context)
        ball = context.known_ball

        # Desperation: ball critically close to goal line — approach ball
        # directly, bypassing state machine target (which may be lateral
        # block or guard and would prevent reaching the ball).
        margin = kit.config.strategy.gk_desperation_clear_margin_m
        own_goal_x = kit.field.own_goal_x()
        if ball.x < own_goal_x + margin:
            raw = kit.motion.approach_target(ball, 0.0, 0.2)
            logger = kit.logger
            if logger is not None:
                logger.info(
                    f"GK desperation approach ball=({ball.x:.3f},{ball.y:.3f})",
                    event="goalkeeper_desperation_approach",
                    ball_x=round(ball.x, 3), ball_y=round(ball.y, 3),
                )
            return self._smooth_target(raw, kit)

        if self._gk_state == self._RUSH_OUT:
            raw = self._rush_out_target(kit, context, ball)
        elif self._gk_state == self._LATERAL:
            raw = self._lateral_target(kit, context, ball)
        else:
            raw = kit.ready_stance.goalkeeper_guard_target(ball)

        return self._smooth_target(raw, kit)

    def wants_to_kick(
        self,
        kit: "SoccerKit",
        context: PlayContext,
    ) -> bool:
        self._ensure_updated(kit, context)
        ball = context.known_ball
        margin = kit.config.strategy.gk_desperation_clear_margin_m

        # Desperation clear always overrides — last line of defence.
        if ball is not None and ball.x < kit.field.own_goal_x() + margin:
            return True

        if self._gk_state != self._RUSH_OUT:
            return False

        # Even in RUSH_OUT, defer if an outfield teammate is closer to the
        # ball (avoids GK–chaser double-kick contention).
        gk_id = kit.config.goalkeeper_player_id()
        game = context.known_game
        robot = context.teammates.get(gk_id)
        if robot is None or robot.pose is None:
            return True
        my_dist = math.hypot(ball.x - robot.pose.x, ball.y - robot.pose.y)

        for tid, trobot in context.teammates.items():
            if (
                tid == gk_id
                or trobot.pose is None
                or not kit.is_player_allowed(game, tid)
            ):
                continue
            teammate_dist = math.hypot(
                ball.x - trobot.pose.x, ball.y - trobot.pose.y,
            )
            if teammate_dist < my_dist + 1.0:
                return False

        return True

    def kick_target(
        self,
        kit: "SoccerKit",
        context: PlayContext,
    ) -> Pose2D:
        ball = context.known_ball

        clear_x = kit.field.opponent_goal_x() - 0.7
        own_goal_x = kit.field.own_goal_x()

        # Desperation clear: ball within margin of goal line — kick as far as
        # possible toward the opponent goal, ignoring direction optimisation.
        desperation_margin = kit.config.strategy.gk_desperation_clear_margin_m
        if ball.x < own_goal_x + desperation_margin:
            return Pose2D(kit.field.opponent_goal_x(), 0.0, kit.field.attack_theta())

        # Defensive fallback if predictor not yet initialised
        if self._predictor is None:
            return Pose2D(clear_x, 0.0, kit.field.attack_theta())

        # Static ball in goal area → force forward center clearance
        ball_speed = math.hypot(self._predictor.smooth_vx, self._predictor.smooth_vy)
        if ball.x < own_goal_x + 0.8 and ball_speed < 0.3:
            return Pose2D(clear_x, 0.0, kit.field.attack_theta())

        keeper_id = kit.config.goalkeeper_player_id()
        robot = context.teammates.get(keeper_id)
        if robot is None or robot.pose is None:
            return Pose2D(clear_x, 0.0, kit.field.attack_theta())

        goal_w = kit.config.goal_width
        candidates = [
            (clear_x, 0.0),                        # opponent half center
            (clear_x, goal_w * 0.5),               # opponent goal top post
            (clear_x, -goal_w * 0.5),              # opponent goal bottom post
            (clear_x, 3.0),                        # top sideline
            (clear_x, -3.0),                       # bottom sideline
        ]
        dir_names = ("center", "top", "bottom", "top_side", "bottom_side")

        # Lock direction for the entire clear cycle
        if self._last_kick_dir_index == -1:
            obstacles = kit.obstacles.opponent_obstacles(context)
            best_index = 0
            best_combined = -float("inf")
            for i, (tx, ty) in enumerate(candidates):
                lane = kit.targeting.lane_clear_score(
                    ball.x, ball.y, tx, ty, obstacles,
                )
                theta = math.atan2(ty - ball.y, tx - ball.x)
                turn = abs(normalize_angle(theta - robot.pose.theta))
                combined = lane - 0.3 * turn
                if combined > best_combined:
                    best_combined = combined
                    best_index = i
            self._last_kick_dir_index = best_index
            dir_name = dir_names[best_index]
            logger = kit.logger
            if logger is not None:
                logger.info(
                    f"GK kick dir: {dir_name} "
                    f"ball=({ball.x:.3f},{ball.y:.3f}) "
                    f"robot_theta={robot.pose.theta:.2f}",
                    event="goalkeeper_kick_direction",
                    direction=dir_name,
                    ball_x=round(ball.x, 3), ball_y=round(ball.y, 3),
                    robot_theta=round(robot.pose.theta, 2),
                )

        return Pose2D(candidates[self._last_kick_dir_index][0],
                      candidates[self._last_kick_dir_index][1],
                      kit.field.attack_theta())

    # ------------------------------------------------------------------
    # State-specific target computations
    # ------------------------------------------------------------------

    def _rush_out_target(
        self,
        kit: "SoccerKit",
        context: PlayContext,
        ball: BallState,
    ) -> Pose2D:
        """Approach the predicted rest point to intercept and clear."""
        rest_x, rest_y = self._predictor.predict_rest_point()

        kt = self.kick_target(kit, context)
        kick_theta = math.atan2(kt.y - rest_y, kt.x - rest_x)
        target = kit.motion.approach_target(
            BallState(x=rest_x, y=rest_y, last_seen_at=ball.last_seen_at),
            kick_theta,
            self._APPROACH_OFFSET,
        )
        target = kit.field.clamp_from_goal_obstructions(target)

        logger = kit.logger
        if logger is not None and not self._state_logged:
            self._state_logged = True
            dir_name = (
                "center" if abs(kt.y) < 1.0
                else "top" if kt.y > 0
                else "bottom"
            )
            logger.info(
                f"GK rush-out plan: dir={dir_name} "
                f"ball=({ball.x:.3f},{ball.y:.3f}) "
                f"rest_pred=({rest_x:.3f},{rest_y:.3f}) "
                f"ball_v=({self._predictor.smooth_vx:.2f},{self._predictor.smooth_vy:.2f}) "
                f"mu={self._predictor.friction:.2f}",
                event="goalkeeper_rush_out",
                direction=dir_name,
                ball_x=round(ball.x, 3), ball_y=round(ball.y, 3),
                rest_x=round(rest_x, 3), rest_y=round(rest_y, 3),
                ball_vx=round(self._predictor.smooth_vx, 2),
                ball_vy=round(self._predictor.smooth_vy, 2),
                friction=round(self._predictor.friction, 2),
            )

        return target

    def _lateral_target(
        self,
        kit: "SoccerKit",
        context: PlayContext,
        ball: BallState,
    ) -> Pose2D:
        """Lateral intercept along the guard-depth line."""
        goal_x = kit.field.own_goal_x()
        goal_hw = kit.config.goal_width / 2.0
        guard_x = goal_x + kit.config.strategy.goalkeeper_guard_depth_m

        crossing = self._predictor.predict_goal_crossing(goal_x, goal_hw)
        if crossing is not None:
            cross_y = max(-goal_hw, min(goal_hw, crossing[1]))
        else:
            cross_y = 0.0

        theta = kit.field.face_ball_theta(guard_x, cross_y, ball)
        target = Pose2D(guard_x, cross_y, theta)

        logger = kit.logger
        if logger is not None and not self._state_logged:
            self._state_logged = True
            t_cross = crossing[0] if crossing is not None else -1.0
            logger.info(
                f"GK lateral block: cross_y={cross_y:.3f} "
                f"t_cross={t_cross:.2f}s "
                f"ball=({ball.x:.3f},{ball.y:.3f})",
                event="goalkeeper_lateral",
                cross_y=round(cross_y, 3),
                t_cross=round(t_cross, 2),
                ball_x=round(ball.x, 3), ball_y=round(ball.y, 3),
            )

        return target

    # ------------------------------------------------------------------
    # Trajectory smoothing layer
    # ------------------------------------------------------------------

    def _smooth_target(self, raw: Pose2D, kit: "SoccerKit") -> Pose2D:
        """Rate-limit target position to prevent sudden jumps on state change."""
        if not self._smooth_init:
            self._smooth_x = raw.x
            self._smooth_y = raw.y
            self._smooth_init = True
            return raw

        max_speed = kit.config.strategy.gk_target_smooth_speed
        dt = 1.0 / kit.config.control_hz
        max_delta = max_speed * dt

        dx = raw.x - self._smooth_x
        dy = raw.y - self._smooth_y
        dist = math.hypot(dx, dy)

        if dist > max_delta and dist > 0.001:
            scale = max_delta / dist
            self._smooth_x += dx * scale
            self._smooth_y += dy * scale
        else:
            self._smooth_x = raw.x
            self._smooth_y = raw.y

        return Pose2D(self._smooth_x, self._smooth_y, raw.theta)

    # ------------------------------------------------------------------
    # Build
    # ------------------------------------------------------------------

    def _guard_reason(self) -> str:
        return "goalkeeper guard"

    def _kick_reason(
        self,
        kit: "SoccerKit",
        target: Pose2D,
    ) -> str:
        return kit.targeting.kick_reason(target, default="goalkeeper clear")

    def build_subtree(
        self,
        kit: "SoccerKit",
        player_id: int,
    ) -> py_trees.behaviour.Behaviour:
        return build_attack_subtree(
            kit,
            player_id,
            AttackSubtreeConfig(
                target_fn=lambda context: self.target(kit, context),
                kick_target_fn=lambda context: self.kick_target(kit, context),
                wants_kick_fn=lambda context: self.wants_to_kick(kit, context),
                reason_fn=self._guard_reason,
                kick_reason_fn=lambda target: self._kick_reason(kit, target),
                hold_vyaw=0.12,
                strafe=True,
                speed_multiplier=kit.config.strategy.goalkeeper_rush_speed_multiplier,
                kick_power=kit.config.strategy.goalkeeper_kick_power,
                lateral_speed=kit.config.strategy.goalkeeper_lateral_speed,
            ),
        )
