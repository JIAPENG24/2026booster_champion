"""Core PLAY subtree leaves: dynamic role assignment, fallback, and role utilities.

Shared PLAY leaves include :class:`AssignRoles`, :class:`IsRole`, and
:class:`WaitForBall`. Role utility leaves and builders include
:class:`MoveToTarget`, :class:`KickAction`, :class:`IsKickWanted`, and
:func:`build_attack_subtree`.

All data is read from the blackboard. Commands are written to
``cmd_key(player_id)`` and finally collected by :class:`CommitTeamCommands`.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

from ..tactics.geometry import clamp

import py_trees

from ..soccer_framework import PlayContext, Pose2D, ReadySlot, RobotCommand
from ..behavior_tree.blackboard import BlackboardKeys, BlackboardClient, cmd_key
from ..behavior_tree.nodes.conditions import IsInKickRange
from .playbook import Playbook, RoleAssignment

if TYPE_CHECKING:
    from ..runtime import SoccerKit


def _read_play_context(
    blackboard: BlackboardClient,
) -> PlayContext | None:
    """Read and narrow the blackboard values needed by PLAY decisions."""

    context = blackboard.read(BlackboardKeys.PLAY_CONTEXT)
    if not isinstance(context, PlayContext):
        return None
    return context


# ----------------------------------------------------------------------
# PLAY dynamic role assignment, written to blackboard
# ----------------------------------------------------------------------


class AssignRoles(py_trees.behaviour.Behaviour):
    """Call :meth:`Playbook.assign_roles` each tick and write the result to ``/team/roles``.

    This is the PLAY subtree's core strategy node. Every :class:`IsRole` condition
    leaf branches from its :class:`RoleAssignment`, so overriding ``Playbook.assign_roles``
    changes the whole team's PLAY role assignment.

    SafetyGuards have already filtered missing GameController and ball data before
    this leaf runs in PLAYING.
    """

    def __init__(self, kit: "SoccerKit", playbook: Playbook):
        super().__init__("AssignRoles")
        self._kit = kit
        self._playbook = playbook
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        context = _read_play_context(self.blackboard)
        if context is None:
            self.blackboard.write(BlackboardKeys.ROLES, RoleAssignment())
            return py_trees.common.Status.SUCCESS
        assignment = self._playbook.assign_roles(context)
        self.blackboard.write(BlackboardKeys.ROLES, assignment)
        # Cache on the kit so role strategies (e.g. SupporterRole) can read the
        # assigned chaser/keeper IDs directly without re-deriving them by scanning
        # teammates — which would otherwise pick another supporter as the chaser
        # when the goalkeeper is the real chaser (danger-zone mutual drift).
        self._kit.current_roles = assignment
        return py_trees.common.Status.SUCCESS


# ----------------------------------------------------------------------
# Common condition leaf: IsRole
# ----------------------------------------------------------------------


class IsRole(py_trees.behaviour.Behaviour):
    """Read ``/team/roles`` and test whether a player currently has the ``expected`` role."""

    def __init__(self, player_id: int, expected: str):
        super().__init__(f"IsRole<{expected}>({player_id})")
        self._player_id = player_id
        self._expected = expected
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        assignment = self.blackboard.read(BlackboardKeys.ROLES)
        if not isinstance(assignment, RoleAssignment):
            return py_trees.common.Status.FAILURE
        if assignment.role_of(self._player_id) != self._expected:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


# ----------------------------------------------------------------------
# Player Selector fallback leaf
# ----------------------------------------------------------------------


class WaitForBall(py_trees.behaviour.Behaviour):
    """Fallback command when no role is assigned; behavior comes from :meth:`Playbook.waiting_command`.

    As the final Player Selector fallback it must always write a command. Even if
    context is absent, it writes a ReadySlot-tagged stop so PlaybookCore does not
    fail into ``StopAll("unsupported state")``.
    """

    def __init__(self, kit: "SoccerKit", playbook: Playbook, player_id: int):
        super().__init__(f"WaitForBall({player_id})")
        self._kit = kit
        self._playbook = playbook
        self._player_id = player_id
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        self._kit.kicker.clear_player(self._player_id)
        context = _read_play_context(self.blackboard)
        if context is None:
            slot = self._kit.config.ready_slot_for_player(self._player_id)
            command = RobotCommand.stop(f"{slot.value} waiting for ball")
        else:
            command = self._playbook.waiting_command(self._player_id, context)
        self.blackboard.write(cmd_key(self._player_id), command)
        return py_trees.common.Status.SUCCESS


# ----------------------------------------------------------------------
# KickAction / IsKickWanted / Role utility leaves
# ----------------------------------------------------------------------

# Callable type aliases used by utility nodes
TargetFn = Callable[[PlayContext], Pose2D]
WantsKickFn = Callable[[PlayContext], bool]
ReasonFn = Callable[[], str]
KickReasonFn = Callable[[Pose2D], str]


def _default_move_reason(player_id: int) -> str:
    return f"role {player_id} move"


def _default_kick_reason(player_id: int) -> str:
    return f"role {player_id} kick"


@dataclass(frozen=True)
class AttackSubtreeConfig:
    """Parameters for the standard attack subtree builder."""

    target_fn: TargetFn
    kick_target_fn: TargetFn
    wants_kick_fn: WantsKickFn | None = None
    reason_fn: ReasonFn | None = None
    kick_reason_fn: KickReasonFn | None = None
    hold_vyaw: float = 0.0
    strafe: bool = False
    speed_multiplier: float = 1.0
    kick_power: float | None = None
    lateral_speed: float | None = None  # Override lateral speed limit when strafing (m/s).


class MoveToTarget(py_trees.behaviour.Behaviour):
    """Call ``target_fn`` each tick to compute a target and issue a move command.

    Kick hysteresis is cleared before every move to avoid leaking last tick's
    kick state into this tick's movement. If ``target_fn`` raises ``ValueError``,
    this leaf returns FAILURE and the parent Selector falls back.
    """

    def __init__(
        self,
        kit: "SoccerKit",
        player_id: int,
        target_fn: TargetFn,
        *,
        reason_fn: ReasonFn | None = None,
        hold_vyaw: float = 0.0,
        speed_multiplier: float = 1.0,
        strafe: bool = False,
        lateral_speed: float | None = None,
    ):
        super().__init__(f"MoveToTarget({player_id})")
        self._kit = kit
        self._player_id = player_id
        self._target_fn = target_fn
        self._reason_fn: ReasonFn = reason_fn or (
            lambda: _default_move_reason(player_id)
        )
        self._hold_vyaw = hold_vyaw
        self._speed_multiplier = speed_multiplier
        self._strafe = strafe
        self._lateral_speed = lateral_speed
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        context = _read_play_context(self.blackboard)
        if context is None:
            return py_trees.common.Status.FAILURE
        try:
            target = self._target_fn(context)
        except ValueError:
            return py_trees.common.Status.FAILURE

        kit = self._kit
        player_id = self._player_id
        kit.kicker.clear_player(player_id)
        command = kit.motion.move_to_target(
            player_id,
            context,
            target,
            self._reason_fn(),
            hold_vyaw=self._hold_vyaw,
            speed_multiplier=self._speed_multiplier,
            strafe=self._strafe,
            lateral_speed=self._lateral_speed,
        )
        self.blackboard.write(cmd_key(player_id), command)
        return py_trees.common.Status.SUCCESS


class KickAction(py_trees.behaviour.Behaviour):
    """Call ``kick_target_fn`` each tick to compute the aim point and issue a kick command.

    Returns FAILURE when ``kick_target_fn`` raises ``ValueError``.
    """

    def __init__(
        self,
        kit: "SoccerKit",
        player_id: int,
        kick_target_fn: TargetFn,
        *,
        reason_fn: KickReasonFn | None = None,
        power: float | None = None,
    ):
        super().__init__(f"KickAction({player_id})")
        self._kit = kit
        self._player_id = player_id
        self._kick_target_fn = kick_target_fn
        self._reason_fn = reason_fn
        self._power = power
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        context = _read_play_context(self.blackboard)
        if context is None:
            return py_trees.common.Status.FAILURE
        try:
            kt = self._kick_target_fn(context)
        except ValueError:
            return py_trees.common.Status.FAILURE

        ball = context.known_ball
        kick_theta = math.atan2(kt.y - ball.y, kt.x - ball.x)
        kit = self._kit
        player_id = self._player_id
        reason = (
            self._reason_fn(kt)
            if self._reason_fn is not None
            else _default_kick_reason(player_id)
        )
        command = kit.motion.kick_command(
            player_id,
            context,
            kick_theta,
            reason,
            power=self._power,
        )
        self.blackboard.write(cmd_key(player_id), command)
        return py_trees.common.Status.SUCCESS


class IsKickWanted(py_trees.behaviour.Behaviour):
    """Condition leaf that calls ``wants_kick_fn`` to decide whether this tick wants a kick."""

    def __init__(
        self,
        kit: "SoccerKit",
        player_id: int,
        wants_kick_fn: WantsKickFn,
    ):
        super().__init__(f"IsKickWanted({player_id})")
        self._kit = kit
        self._player_id = player_id
        self._wants_kick_fn = wants_kick_fn
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        context = _read_play_context(self.blackboard)
        if context is None:
            return py_trees.common.Status.FAILURE
        try:
            if self._wants_kick_fn(context):
                return py_trees.common.Status.SUCCESS
        except ValueError:
            pass
        return py_trees.common.Status.FAILURE


# ----------------------------------------------------------------------
# Default assembly factory: build_attack_subtree
# ----------------------------------------------------------------------


def build_attack_subtree(
    kit: "SoccerKit",
    player_id: int,
    config: AttackSubtreeConfig,
) -> py_trees.behaviour.Behaviour:
    """Assemble the standard attack subtree: conditional kick, otherwise move.

    Shape::

    Selector(Attack)
    |-- Sequence(KickBranch)
    |   |-- IsInKickRange
    |   `-- KickAction
    `-- MoveToTarget

    When ``config.wants_kick_fn`` is ``None``, :class:`IsKickWanted` is omitted,
    equivalent to "always willing to kick". SafetyGuards ensure visible ball data
    before this PLAY subtree runs.
    """

    kick_children: list[py_trees.behaviour.Behaviour] = []
    if config.wants_kick_fn is not None:
        kick_children.append(IsKickWanted(kit, player_id, config.wants_kick_fn))
    kick_children.extend(
        [
            IsInKickRange(player_id, kit.kicker),
            KickAction(
                kit,
                player_id,
                config.kick_target_fn,
                reason_fn=config.kick_reason_fn,
                power=config.kick_power,
            ),
        ],
    )

    return py_trees.composites.Selector(
        name=f"Attack({player_id})",
        memory=False,
        children=[
            py_trees.composites.Sequence(
                name=f"KickBranch({player_id})",
                memory=False,
                children=kick_children,
            ),
            MoveToTarget(
                kit,
                player_id,
                config.target_fn,
                reason_fn=config.reason_fn,
                hold_vyaw=config.hold_vyaw,
                strafe=config.strafe,
                speed_multiplier=config.speed_multiplier,
                lateral_speed=config.lateral_speed,
            ),
        ],
    )


# ----------------------------------------------------------------------
# Our-kickoff actions
# ----------------------------------------------------------------------


class SetPhase(py_trees.behaviour.Behaviour):
    """Write ``KICKOFF_PHASE`` to the given value and return SUCCESS."""

    def __init__(self, kit: "SoccerKit", phase: int):
        super().__init__(f"SetPhase({phase})")
        self._kit = kit
        self._phase = phase
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        self.blackboard.write(BlackboardKeys.KICKOFF_PHASE, self._phase)
        return py_trees.common.Status.SUCCESS


class InitiateKickoff(py_trees.behaviour.Behaviour):
    """Write the initial ball position and start time for our-kickoff phase.

    Idempotent: only writes on the first tick (when ball record is None).
    """

    def __init__(self, kit: "SoccerKit"):
        super().__init__("InitiateKickoff")
        self._kit = kit
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        if self.blackboard.read(BlackboardKeys.KICKOFF_BALL_X) is not None:
            return py_trees.common.Status.SUCCESS
        context = _read_play_context(self.blackboard)
        now = self.blackboard.read(BlackboardKeys.NOW)
        if context is None or context.ball is None or now is None:
            return py_trees.common.Status.FAILURE
        self.blackboard.write(BlackboardKeys.KICKOFF_BALL_X, context.ball.x)
        self.blackboard.write(BlackboardKeys.KICKOFF_BALL_Y, context.ball.y)
        self.blackboard.write(BlackboardKeys.KICKOFF_STARTED_AT, now)
        return py_trees.common.Status.SUCCESS


class AssignFixedRoles(py_trees.behaviour.Behaviour):
    """Write a fixed role mapping to the blackboard, bypassing ``Playbook.assign_roles``."""

    def __init__(
        self,
        kit: "SoccerKit",
        *,
        center_role: str,
        side_role: str,
        keeper_role: str,
    ):
        super().__init__("AssignFixedRoles")
        self._kit = kit
        self._center_role = center_role
        self._side_role = side_role
        self._keeper_role = keeper_role
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        mapping: dict[int, str] = {}
        for player_id in self._kit.config.player_ids:
            slot = self._kit.config.ready_slot_for_player(player_id)
            if slot == ReadySlot.CENTER:
                mapping[player_id] = self._center_role
            elif slot == ReadySlot.SIDE:
                mapping[player_id] = self._side_role
            else:
                mapping[player_id] = self._keeper_role
        self.blackboard.write(BlackboardKeys.ROLES, RoleAssignment(mapping))
        return py_trees.common.Status.SUCCESS


class KickAtAngle(py_trees.behaviour.Behaviour):
    """Kick in a fixed field-frame direction (radians).

    ``power`` overrides the default ``soccer_kick_power``; ``None`` uses the default.
    """

    def __init__(
        self,
        kit: "SoccerKit",
        player_id: int,
        angle_rad: float,
        *,
        power: float | None = None,
    ):
        super().__init__(f"KickAtAngle({player_id},{angle_rad:.3f})")
        self._kit = kit
        self._player_id = player_id
        self._angle_rad = angle_rad
        self._power = power
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        context = _read_play_context(self.blackboard)
        if context is None:
            return py_trees.common.Status.FAILURE
        kit = self._kit
        player_id = self._player_id
        command = kit.motion.kick_command(
            player_id,
            context,
            self._angle_rad,
            "kickoff kick",
            power=self._power,
        )
        self.blackboard.write(cmd_key(player_id), command)
        return py_trees.common.Status.SUCCESS





class MoveToLandingPoint(py_trees.behaviour.Behaviour):
    """Move to a fixed landing point during kickoff, facing the ball."""

    def __init__(
        self,
        kit: "SoccerKit",
        player_id: int,
        landing_x: float,
        landing_y: float,
        *,
        speed_multiplier: float = 1.0,
    ):
        super().__init__(f"MoveToLandingPoint({player_id})")
        self._kit = kit
        self._player_id = player_id
        self._landing_x = landing_x
        self._landing_y = landing_y
        self._speed_multiplier = speed_multiplier
        self.blackboard = BlackboardClient(name=self.name)

    def update(self) -> py_trees.common.Status:
        context = _read_play_context(self.blackboard)
        if context is None:
            return py_trees.common.Status.FAILURE
        ball = context.ball
        kit = self._kit
        player_id = self._player_id
        theta = kit.field.face_ball_theta(
            self._landing_x, self._landing_y, ball,
        )
        target = Pose2D(self._landing_x, self._landing_y, theta)
        kit.kicker.clear_player(player_id)
        command = kit.motion.move_to_target(
            player_id,
            context,
            target,
            "kickoff chase",
            hold_vyaw=0.0,
            speed_multiplier=self._speed_multiplier,
        )
        self.blackboard.write(cmd_key(player_id), command)
        return py_trees.common.Status.SUCCESS



