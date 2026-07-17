"""Data-layer leaves that write external inputs and shared state to the blackboard each tick.

``UpdateGameState``
The tree root chains these nodes in a ``Sequence`` because they depend on each other:
``UpdateClock`` writes time, ``UpdatePlayContext`` reads callbacks, freshness filters run,
and ``UpdateRobotStatus`` pulls hardware status.

PLAY-stage role assignment used to live here (old ``UpdateChaser``), but now
belongs to :class:`src.play.nodes.AssignRoles` so it only runs when play actually starts.

To add a new global input, add a ``_DataLeaf`` subclass and mount it in the DataLayer ``Sequence``.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import py_trees

from ...soccer_framework import (
    GameState,
    RobotRuntimeStatus,
    PlayContext,
    PlayContextProvider,
)
from ..blackboard import BlackboardKeys, BlackboardClient, robot_status_key

if TYPE_CHECKING:
    from ...runtime import SoccerKit


_GAME_STATE_STALE_SEC = 2.0


class _DataLeaf(py_trees.behaviour.Behaviour):
    """Shared base for data-layer leaves; always returns SUCCESS."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = BlackboardClient(name=name)


class UpdateClock(_DataLeaf):
    """Write current time to ``/clock/now`` and reset frame-level flags."""

    def __init__(self, get_now: Callable[[], float]):
        super().__init__("UpdateClock")
        self._get_now = get_now

    def update(self) -> py_trees.common.Status:
        self.blackboard.write(BlackboardKeys.NOW, self._get_now())
        self.blackboard.write(BlackboardKeys.SAFETY_ACTIVE, False)
        self.blackboard.write(BlackboardKeys.READY_TARGETS, None)
        # Keep KICKOFF_PHASE across ticks; reset to 0 when not PLAYING.
        game = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        if isinstance(game, PlayContext):
            gs = game.game_state
            if gs is not None and gs.state != GameState.PLAYING:
                self.blackboard.write(BlackboardKeys.KICKOFF_PHASE, 0)
                self.blackboard.write(BlackboardKeys.KICKOFF_BALL_X, None)
                self.blackboard.write(BlackboardKeys.KICKOFF_BALL_Y, None)
                self.blackboard.write(BlackboardKeys.KICKOFF_KICK_AT, None)
                self.blackboard.write(BlackboardKeys.KICKOFF_EXIT_REQUESTED_AT, None)
                self.blackboard.write(BlackboardKeys.KICKOFF_PHASE_ENTERED_AT, None)
                self.blackboard.write(BlackboardKeys.OPP_KICKOFF_WAS_ACTIVE, False)
        return py_trees.common.Status.SUCCESS


class UpdatePlayContext(_DataLeaf):
    """Snapshot the context from :class:`PlayContextProvider` directly into ``/play_context``.

    Holding the provider here avoids an extra callback chain through runtime,
    strategy, tree, and a temporary cached context.
    """

    def __init__(self, provider: PlayContextProvider):
        super().__init__("UpdatePlayContext")
        self._provider = provider

    def update(self) -> py_trees.common.Status:
        context = self._provider.get_snapshot()
        self.blackboard.write(BlackboardKeys.PLAY_CONTEXT, context)
        return py_trees.common.Status.SUCCESS


class UpdateGameState(_DataLeaf):
    """Clear stale GameControlState in place using the data-layer watchdog.

    Freshness filtering is centralized in the data layer and mutates
    ``context.game_state`` in place, so strategy code reads the filtered value without a separate key.
    These thresholds protect against stopped publishers or blocked callbacks; they are
    not tactic tuning knobs.

    ``last_seen_at == 0.0`` means no topic callback has written it yet, often in
    tests, so it is not filtered to match the old ``last_topic_at <= 0.0`` behavior.
    """

    def __init__(self):
        super().__init__("UpdateGameState")

    def update(self) -> py_trees.common.Status:
        context = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        now = self.blackboard.read(BlackboardKeys.NOW)
        if not isinstance(context, PlayContext) or now is None:
            return py_trees.common.Status.SUCCESS
        gs = context.game_state
        if gs is not None and not gs.is_recent(now, _GAME_STATE_STALE_SEC):
            context.game_state = None
        return py_trees.common.Status.SUCCESS


class UpdateRecentBall(_DataLeaf):
    """Filter stale ball using the LKG buffer for transparent grace-period extrapolation.

    Delegates freshness + grace logic to ``kit.ball_lkg``. When the raw ball
    is stale, the buffer returns a constant-velocity extrapolation during the
    grace window (confidence=0.5) so SafetyGuards' IsBallKnown check passes
    and the team keeps playing. After grace expires, the buffer returns None
    and StopAll fires normally.

    AR-02 compliant: the blackboard (context.ball) remains the sole data source.
    AR-08 compliant: SafetyGuards is not bypassed — it fires after grace.
    """

    def __init__(self, kit: "SoccerKit"):
        super().__init__("UpdateRecentBall")
        self._kit = kit
        self._last_staleness_log_at: float = 0.0

    def _staleness_state(self, age: float) -> str:
        fresh = self._kit.config.strategy.ball_fresh_sec
        grace = self._kit.config.strategy.ball_stale_grace_sec
        if age <= fresh:
            return "fresh"
        if age <= fresh + grace:
            return "grace"
        return "stale"

    def update(self) -> py_trees.common.Status:
        context = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        now = self.blackboard.read(BlackboardKeys.NOW)
        if not isinstance(context, PlayContext) or now is None:
            return py_trees.common.Status.SUCCESS
        context.ball = self._kit.ball_lkg.update(context.ball, now)
        # Periodic ball staleness summary (0.5 Hz) for LKG fix verification.
        if now - self._last_staleness_log_at >= 2.0:
            self._last_staleness_log_at = now
            logger = self._kit.logger
            if logger is not None:
                ball = context.ball
                if ball is not None:
                    age = now - ball.last_seen_at
                    state = self._staleness_state(age)
                    confidence = ball.confidence if hasattr(ball, "confidence") else 1.0
                else:
                    state = "stale"
                    age = -1.0
                    confidence = 0.0
                if ball is not None:
                    msg = (
                        f"ball staleness state={state} "
                        f"age={age:.3f}s conf={confidence:.2f} "
                        f"pos=({ball.x:.3f},{ball.y:.3f})"
                    )
                else:
                    msg = "ball staleness state=stale no data"
                logger.info(
                    msg,
                    event="ball_staleness",
                    state=state,
                    age_sec=round(age, 3),
                    confidence=round(confidence, 2),
                    ball_x=round(ball.x, 3) if ball is not None else None,
                    ball_y=round(ball.y, 3) if ball is not None else None,
                )
        return py_trees.common.Status.SUCCESS


class UpdateRobotPoses(_DataLeaf):
    """Filter stale teammate/opponent poses via per-team LKG buffers.

    Delegates freshness + grace logic to ``kit.pose_lkg_teammates`` and
    ``kit.pose_lkg_opponents``. When a raw pose goes stale, the buffer returns
    a constant-velocity extrapolation (x, y, and theta) during the grace window
    so the motion controller and role assignment keep operating for that
    player instead of emitting a premature ``waiting for pose`` stop. After
    grace expires, the buffer clears the pose to None and existing
    ``if robot.pose is None`` checks in strategy and motion code take over.

    AR-02 compliant: the blackboard (context.teammates/opponents) remains the
    sole data source. Grace filtering mutates poses in place on the
    provider-snapshotted context, so provider internals are not polluted.
    """

    def __init__(self, kit: "SoccerKit"):
        super().__init__("UpdateRobotPoses")
        self._kit = kit
        self._last_staleness_log_at: float = 0.0

    @staticmethod
    def _count_staleness(
        robots: dict, now: float, fresh_sec: float, grace_sec: float,
    ) -> dict[str, int]:
        counts = {"fresh": 0, "grace": 0, "stale": 0}
        for robot in robots.values():
            pose = robot.pose
            if pose is None:
                counts["stale"] += 1
                continue
            age = now - robot.last_seen_at
            if age <= fresh_sec:
                counts["fresh"] += 1
            elif age <= fresh_sec + grace_sec:
                counts["grace"] += 1
            else:
                counts["stale"] += 1
        return counts

    def update(self) -> py_trees.common.Status:
        context = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        now = self.blackboard.read(BlackboardKeys.NOW)
        if not isinstance(context, PlayContext) or now is None:
            return py_trees.common.Status.SUCCESS
        self._kit.pose_lkg_teammates.update(context.teammates, now)
        self._kit.pose_lkg_opponents.update(context.opponents, now)
        # Periodic pose staleness summary (0.5 Hz) for LKG fix verification.
        if now - self._last_staleness_log_at >= 2.0:
            self._last_staleness_log_at = now
            logger = self._kit.logger
            if logger is not None:
                strat = self._kit.config.strategy
                tm = self._count_staleness(
                    context.teammates, now,
                    strat.robot_pose_fresh_sec, strat.robot_pose_stale_grace_sec,
                )
                opp = self._count_staleness(
                    context.opponents, now,
                    strat.robot_pose_fresh_sec, strat.robot_pose_stale_grace_sec,
                )
                logger.info(
                    f"pose staleness teammates_f={tm['fresh']} g={tm['grace']} s={tm['stale']} "
                    f"opponents_f={opp['fresh']} g={opp['grace']} s={opp['stale']}",
                    event="pose_staleness",
                    teammates=tm,
                    opponents=opp,
                )
        return py_trees.common.Status.SUCCESS


class UpdateRobotStatus(_DataLeaf):
    """Pull one player's hardware status onto the blackboard.

    This is read-only: it calls throttled ``poll_runtime_status`` via
    :class:`RobotServices`, writes ``/robot_status/{player_id}``, and lets SafetyOverrides own side effects.

    Without bound services, such as tests or dry-run, it writes a default
    :class:`RobotRuntimeStatus` so guards naturally take the normal branch.
    """

    def __init__(self, kit: "SoccerKit", player_id: int):
        super().__init__(f"UpdateRobotStatus({player_id})")
        self._kit = kit
        self._player_id = player_id

    def update(self) -> py_trees.common.Status:
        now = self.blackboard.read(BlackboardKeys.NOW)
        services = self._kit.robot_services()
        if services is None or now is None:
            status = RobotRuntimeStatus()
        else:
            status = services.poll_runtime_status(self._player_id, now)
        self.blackboard.write(robot_status_key(self._player_id), status)
        return py_trees.common.Status.SUCCESS
