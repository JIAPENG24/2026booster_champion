"""PLAY-stage subtree factory; a complete playbook map for newcomers.

Shape:

Sequence(PlayingPhase)
|-- IsGameInState(PLAYING)
`-- Selector(KickoffPhase)
    |-- Sequence(ActiveKickoff)           Phase 1: our kickoff, pre-kick
    |   |-- IsInPhase(1)
    |   |-- InitiateKickoff
    |   `-- Parallel(KickoffRoles)
    |       |-- Kicker/CENTER  (approach + 45° kick)
    |       |-- Chaser/SIDE   (move to predicted landing point)
    |       `-- Goalkeeper/KEEPER (normal guard)
    |
    |-- Sequence(RoleLockPlay)            Phase 2: ball kicked, fixed roles
    |   |-- IsInPhase(2)
    |   |-- AssignFixedRoles: side→chaser, center→supporter, keeper→goalkeeper
    |   `-- Parallel(Roles)  (same existing role subtrees)
    |       |-- Selector(Player(1))
    |       |-- Selector(Player(2))
    |       `-- Selector(Player(3))
    |
    `-- Sequence(NormalPlay)              Phase 0: normal DefaultPlaybook
        |-- AssignRoles
        `-- Parallel(Roles)  (same existing role subtrees)

The phase is managed by ``PlayKickoffController`` so transitions are clean.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import py_trees

from ..soccer_framework import GameState, PlayContext, ReadySlot, SetPlay
from ..behavior_tree.nodes.actions import (
    AvoidOpponentRestart,
    GoReadyTarget,
)
from ..behavior_tree.nodes.conditions import (
    IsGameInState,
    IsInKickRange,
    IsOpponentKickoffActive,
    IsOpponentRestartActive,
    IsInPhase,
)
from ..behavior_tree.blackboard import BlackboardKeys, BlackboardClient
from .nodes import (
    AssignFixedRoles,
    AssignRoles,
    InitiateKickoff,
    IsRole,
    KickAtAngle,
    MoveToLandingPoint,
    MoveToTarget,
    WaitForBall,
    )
from .playbook import Playbook

if TYPE_CHECKING:
    from ..runtime import SoccerKit


_KICKOFF_ANGLE = 0.785  # 45 degrees
_KICKOFF_LANDING_DIST = 2.5  # predicted landing distance in meters


def _create_role_subtree(
    kit: "SoccerKit",
    playbook: Playbook,
    player_id: int,
) -> py_trees.behaviour.Behaviour:
    """Per-player subtree that selects a branch by dynamic role.

    Branch order equals ``playbook.role_registry`` registration order, with WaitForBall as the final fallback.
    """

    branches: list[py_trees.behaviour.Behaviour] = []

    kickoff_hold = py_trees.composites.Sequence(
        name=f"KickoffHold({player_id})",
        memory=False,
        children=[
            IsOpponentKickoffActive(kit),
            GoReadyTarget(kit, player_id),
        ],
    )
    branches.append(kickoff_hold)

    personal_avoid_guard = py_trees.composites.Sequence(
        name=f"PenaltyAvoid({player_id})",
        memory=False,
        children=[
            IsOpponentRestartActive(kit),
            AvoidOpponentRestart(kit, player_id),
        ],
    )
    branches.append(personal_avoid_guard)

    for role in playbook.role_registry:
        branches.append(
            py_trees.composites.Sequence(
                name=f"As{role.name.capitalize()}({player_id})",
                memory=False,
                children=[
                    IsRole(player_id, role.name),
                    role.build_subtree(kit, player_id),
                ],
            )
        )
    branches.append(WaitForBall(kit, playbook, player_id))

    return py_trees.composites.Selector(
        name=f"Player({player_id})",
        memory=False,
        children=branches,
    )


def _create_normal_roles_parallel(
    kit: "SoccerKit",
    playbook: Playbook,
) -> py_trees.behaviour.Behaviour:
    """Parallel ``Roles`` used by both RoleLockPlay and NormalPlay."""
    return py_trees.composites.Parallel(
        name="Roles",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=[
            _create_role_subtree(kit, playbook, player_id)
            for player_id in kit.config.player_ids
        ],
    )


# ----------------------------------------------------------------------
# Phase 1: ActiveKickoff per-player slot-based behavior
# ----------------------------------------------------------------------


def _create_active_kickoff_roles(
    kit: "SoccerKit",
) -> py_trees.behaviour.Behaviour:
    """Slot-based per-player subtrees for Phase 1 ActiveKickoff.

    CENTER: approach ball, kick at 45°.
    SIDE:   move to predicted landing point.
    KEEPER: normal goalkeeper guard.
    """

    def _build_kicker(kit: "SoccerKit", pid: int) -> py_trees.behaviour.Behaviour:
        target_fn = lambda ctx: kit.motion.approach_target(
            ctx.known_ball, _KICKOFF_ANGLE, 0.4,
        )
        return py_trees.composites.Selector(
            name=f"Kicker({pid})",
            memory=False,
            children=[
                py_trees.composites.Sequence(
                    name=f"KickBranch({pid})",
                    memory=False,
                    children=[
                        IsInKickRange(pid, kit.kicker),
                        KickAtAngle(kit, pid, _KICKOFF_ANGLE,
                                    power=kit.config.strategy.kickoff_kick_power),
                    ],
                ),
                MoveToTarget(
                    kit, pid, target_fn,
                    reason_fn=lambda: "kicker approach",
                ),
            ],
        )

    def _build_chaser(kit: "SoccerKit", pid: int) -> py_trees.behaviour.Behaviour:
        landing_x = math.cos(_KICKOFF_ANGLE) * _KICKOFF_LANDING_DIST
        landing_y = math.sin(_KICKOFF_ANGLE) * _KICKOFF_LANDING_DIST
        return MoveToLandingPoint(kit, pid, landing_x, landing_y,
                                  speed_multiplier=1.3)

    def _build_keeper(kit: "SoccerKit", pid: int) -> py_trees.behaviour.Behaviour:
        target_fn = lambda ctx: kit.ready_stance.goalkeeper_guard_target(
            ctx.ball,
        )
        return MoveToTarget(
            kit, pid, target_fn,
            reason_fn=lambda: "kickoff keeper",
            hold_vyaw=0.12,
        )

    children: list[py_trees.behaviour.Behaviour] = []
    for player_id in kit.config.player_ids:
        slot = kit.config.ready_slot_for_player(player_id)
        if slot == ReadySlot.CENTER:
            children.append(_build_kicker(kit, player_id))
        elif slot == ReadySlot.SIDE:
            children.append(_build_chaser(kit, player_id))
        else:
            children.append(_build_keeper(kit, player_id))

    return py_trees.composites.Parallel(
        name="KickoffRoles",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=children,
    )


# ----------------------------------------------------------------------
# Phase 3: OppKickoffDefense per-player slot-based behavior
# ----------------------------------------------------------------------


def _create_opp_defense_roles(
    kit: "SoccerKit",
) -> py_trees.behaviour.Behaviour:
    """Slot-based per-player subtrees for Phase 3 OppKickoffDefense.

    Each player approaches the ball at a different speed multiplier:
    CENTER: 0.5x  — intercept.
    SIDE:   1.3x  — press aggressively.
    KEEPER: 1.0x  — press actively.
    """

    def _build_approach(kit: "SoccerKit", pid: int, speed: float) -> py_trees.behaviour.Behaviour:
        target_fn = lambda ctx: kit.motion.approach_target(
            ctx.known_ball, kit.field.attack_theta(), 0.4,
        )
        return MoveToTarget(
            kit, pid, target_fn,
            reason_fn=lambda: "opp approach",
            speed_multiplier=speed,
        )

    children: list[py_trees.behaviour.Behaviour] = []
    for player_id in kit.config.player_ids:
        slot = kit.config.ready_slot_for_player(player_id)
        if slot == ReadySlot.CENTER:
            children.append(_build_approach(kit, player_id, 0.5))
        elif slot == ReadySlot.SIDE:
            children.append(_build_approach(kit, player_id, 1.3))
        else:
            children.append(_build_approach(kit, player_id, 1.0))

    return py_trees.composites.Parallel(
        name="OppDefRoles",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=children,
    )


# ----------------------------------------------------------------------
# Phase controller: drives KICKOFF_PHASE transitions
# ----------------------------------------------------------------------


class PlayKickoffController(py_trees.behaviour.Behaviour):
    """Runs every tick before the phase selector to manage ``KICKOFF_PHASE``.

    Transitions:
    - Phase 0/1/2 → 3: opponent kickoff just ended (secondary_time dropped to 0)
    - Phase 0 → 1: our kickoff is active (state=PLAYING, kicking_team=us, ball near center)
    - Phase 1 → 2: ball has moved > 0.15 m (center kicked)
    - Phase 2 → 0: side close to ball or 2 s timeout
    - Phase 3 → 0: our player touched ball or 5 s timeout
    """

    def __init__(self, kit: "SoccerKit"):
        super().__init__("PlayKickoffController")
        self._kit = kit
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        phase = self.blackboard.read(BlackboardKeys.KICKOFF_PHASE)
        game = self._read_game()
        ball = self._read_ball()
        now = self.blackboard.read(BlackboardKeys.NOW)
        context = self._read_context()

        if game is None:
            return py_trees.common.Status.SUCCESS

        # Track opponent kickoff active state (run every tick regardless of phase)
        opp_was_active = bool(self.blackboard.read(BlackboardKeys.OPP_KICKOFF_WAS_ACTIVE))
        opp_now_active = (
            game.state == GameState.PLAYING
            and game.set_play == SetPlay.NONE
            and game.secondary_time > 0
            and game.has_kicking_team()
            and game.kicking_team != self._kit.config.team_id
        )
        self.blackboard.write(BlackboardKeys.OPP_KICKOFF_WAS_ACTIVE, opp_now_active)

        # Falling edge: opponent just touched the ball → enter Phase 3
        if opp_was_active and not opp_now_active and phase != 3:
            self.blackboard.write(BlackboardKeys.KICKOFF_PHASE, 3)
            self.blackboard.write(BlackboardKeys.KICKOFF_PHASE_ENTERED_AT, now)
            logger = self._kit.logger
            if logger is not None and ball is not None:
                players: list[dict[str, object]] = []
                parts: list[str] = [f"ball=({ball.x:.3f},{ball.y:.3f})"]
                for pid in self._kit.config.player_ids:
                    robot = context.teammates.get(pid) if context is not None else None
                    if robot is not None and robot.pose is not None:
                        slot = self._kit.config.ready_slot_for_player(pid)
                        players.append({
                            "id": pid, "slot": slot.value,
                            "x": round(robot.pose.x, 2), "y": round(robot.pose.y, 2),
                        })
                        parts.append(
                            f"p{pid}:({robot.pose.x:.2f},{robot.pose.y:.2f})"
                        )
                logger.info(
                    "opponent kickoff transition phase=3 " + " ".join(parts),
                    event="opponent_kickoff_transition",
                    phase=3,
                    ball_x=round(ball.x, 3),
                    ball_y=round(ball.y, 3),
                    players=players,
                )
            return py_trees.common.Status.SUCCESS

        # Phase 3 → 0: our player touched ball or 5 s timeout
        if phase == 3:
            if context is not None and now is not None:
                phase_entered = self.blackboard.read(BlackboardKeys.KICKOFF_PHASE_ENTERED_AT)
                if phase_entered is not None and now - phase_entered > 5.0:
                    self.blackboard.write(BlackboardKeys.KICKOFF_PHASE, 0)
                    logger = self._kit.logger
                    if logger is not None and ball is not None:
                        logger.info(
                            f"opponent kickoff transition phase=0 reason=timeout "
                            f"duration={now - phase_entered:.2f}s "
                            f"ball=({ball.x:.3f},{ball.y:.3f})",
                            event="opponent_kickoff_transition",
                            phase=0,
                            reason="timeout",
                            duration_sec=round(now - phase_entered, 2),
                            ball_x=round(ball.x, 3),
                            ball_y=round(ball.y, 3),
                        )
                    return py_trees.common.Status.SUCCESS

                if ball is not None:
                    for pid in self._kit.config.player_ids:
                        robot = context.teammates.get(pid)
                        if robot is not None and robot.pose is not None:
                            dist = math.hypot(robot.pose.x - ball.x, robot.pose.y - ball.y)
                            if dist < 0.3:
                                self.blackboard.write(BlackboardKeys.KICKOFF_PHASE, 0)
                                logger = self._kit.logger
                                if logger is not None:
                                    logger.info(
                                        f"opponent kickoff transition phase=0 "
                                        f"reason=player_{pid}_touched "
                                        f"duration={now - phase_entered:.2f}s "
                                        f"ball=({ball.x:.3f},{ball.y:.3f})",
                                        event="opponent_kickoff_transition",
                                        phase=0,
                                        reason=f"player_{pid}_touched",
                                        duration_sec=round(now - phase_entered, 2),
                                        ball_x=round(ball.x, 3),
                                        ball_y=round(ball.y, 3),
                                    )
                                return py_trees.common.Status.SUCCESS

            return py_trees.common.Status.SUCCESS

        # Phase 0 → 1: our kickoff detected
        if phase == 0 and self._is_our_kickoff_active(game, ball):
            self.blackboard.write(BlackboardKeys.KICKOFF_PHASE, 1)
            self.blackboard.write(BlackboardKeys.KICKOFF_PHASE_ENTERED_AT, now)
            self.blackboard.write(BlackboardKeys.KICKOFF_KICK_AT, None)
            self.blackboard.write(BlackboardKeys.KICKOFF_EXIT_REQUESTED_AT, None)
            return py_trees.common.Status.SUCCESS

        # Phase 1 → 2: ball moved (0.5 s delay after first detection)
        if phase == 1:
            init_x = self.blackboard.read(BlackboardKeys.KICKOFF_BALL_X)
            init_y = self.blackboard.read(BlackboardKeys.KICKOFF_BALL_Y)
            if ball is not None and init_x is not None and init_y is not None:
                dist = math.hypot(ball.x - init_x, ball.y - init_y)
                if dist > 0.15:
                    kick_at = self.blackboard.read(BlackboardKeys.KICKOFF_KICK_AT)
                    if kick_at is None:
                        kick_at = now
                        self.blackboard.write(BlackboardKeys.KICKOFF_KICK_AT, kick_at)
                    if now - kick_at > 0.5:
                        self.blackboard.write(BlackboardKeys.KICKOFF_PHASE, 2)
                        self.blackboard.write(BlackboardKeys.KICKOFF_PHASE_ENTERED_AT, now)
                        self.blackboard.write(BlackboardKeys.KICKOFF_EXIT_REQUESTED_AT, None)
                        return py_trees.common.Status.SUCCESS

        # Phase 2 → 0: side close or timeout (1.0 s exit delay)
        if phase == 2:
            if context is not None and now is not None:
                phase_entered = self.blackboard.read(BlackboardKeys.KICKOFF_PHASE_ENTERED_AT)
                if phase_entered is not None and now - phase_entered > 2.0:
                    self.blackboard.write(BlackboardKeys.KICKOFF_PHASE, 0)
                    return py_trees.common.Status.SUCCESS

                side_pid = self._side_player_id()
                exit_triggered = False
                if side_pid is not None:
                    robot = context.teammates.get(side_pid)
                    if robot is not None and robot.pose is not None and ball is not None:
                        dist = math.hypot(
                            robot.pose.x - ball.x, robot.pose.y - ball.y,
                        )
                        if dist < 0.3:
                            exit_triggered = True

                if exit_triggered:
                    exit_requested = self.blackboard.read(BlackboardKeys.KICKOFF_EXIT_REQUESTED_AT)
                    if exit_requested is None:
                        self.blackboard.write(BlackboardKeys.KICKOFF_EXIT_REQUESTED_AT, now)
                    elif now - exit_requested > 1.0:
                        self.blackboard.write(BlackboardKeys.KICKOFF_PHASE, 0)
                        return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.SUCCESS

    def _is_our_kickoff_active(self, game, ball) -> bool:
        if game.state != GameState.PLAYING:
            return False
        if game.set_play != SetPlay.NONE:
            return False
        if not game.has_kicking_team():
            return False
        if game.kicking_team != self._kit.config.team_id:
            return False
        if ball is None:
            return False
        # Ball should still be near center (not yet kicked)
        if math.hypot(ball.x, ball.y) > 1.0:
            return False
        return True

    def _side_player_id(self) -> int | None:
        for pid in self._kit.config.player_ids:
            if self._kit.config.ready_slot_for_player(pid) == ReadySlot.SIDE:
                return pid
        return None

    def _read_game(self):
        ctx = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        return ctx.game_state if isinstance(ctx, PlayContext) else None

    def _read_ball(self):
        ctx = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        return ctx.ball if isinstance(ctx, PlayContext) else None

    def _read_context(self):
        ctx = self.blackboard.read(BlackboardKeys.PLAY_CONTEXT)
        return ctx if isinstance(ctx, PlayContext) else None


# ----------------------------------------------------------------------
# Main factory
# ----------------------------------------------------------------------


def create_play_subtree(
    kit: "SoccerKit",
    playbook: Playbook,
) -> py_trees.behaviour.Behaviour:
    """PLAY subtree: phase selector over kickoff defense, our kickoff, and normal play."""

    # Phase 3: Opponent-kickoff defense (highest priority)
    opp_defense = py_trees.composites.Sequence(
        name="OppKickoffDefense",
        memory=False,
        children=[
            IsInPhase(3),
            _create_opp_defense_roles(kit),
        ],
    )

    # Phase 1: Active kickoff before ball is kicked
    # Phase transition 1→2 is handled by PlayKickoffController.
    active_kickoff = py_trees.composites.Sequence(
        name="ActiveKickoff",
        memory=False,
        children=[
            IsInPhase(1),
            InitiateKickoff(kit),
            _create_active_kickoff_roles(kit),
        ],
    )

    # Phase 2: Role-lock after kick
    # Uses its own Roles parallel instance (py_trees forbids shared parent nodes).
    role_lock = py_trees.composites.Sequence(
        name="RoleLockPlay",
        memory=False,
        children=[
            IsInPhase(2),
            AssignFixedRoles(
                kit,
                center_role="supporter",
                side_role="chaser",
                keeper_role="goalkeeper",
            ),
            _create_normal_roles_parallel(kit, playbook),
        ],
    )

    # Phase 3: Normal DefaultPlaybook roles
    normal_play = py_trees.composites.Sequence(
        name="NormalPlay",
        memory=False,
        children=[
            AssignRoles(kit, playbook),
            _create_normal_roles_parallel(kit, playbook),
        ],
    )

    # Phase controller + selector
    # Priority: OppDefense(3) > ActiveKickoff(1) > RoleLock(2) > NormalPlay(0)
    phase_branches = py_trees.composites.Selector(
        name="KickoffPhase",
        memory=False,
        children=[opp_defense, active_kickoff, role_lock, normal_play],
    )

    return py_trees.composites.Sequence(
        name="PlayingPhase",
        memory=False,
        children=[
            IsGameInState(GameState.PLAYING),
            PlayKickoffController(kit),
            phase_branches,
        ],
    )
