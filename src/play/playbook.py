"""PLAY-stage strategy entry point; competitors usually edit this file first.

:class:`RoleAssignment` snapshots "who does what" once per tick and stores a
``player_id -> role_name`` mapping that can hold custom roles. :class:`Playbook`
centralizes competitor-overridable PLAY decisions and explicitly registers roles
through :meth:`register_role`.

:class:`DefaultPlaybook` is the template default. It registers the chaser,
supporter, and goalkeeper roles; fixed starting slots use ``ReadySlot`` for
non-PLAY branches. To change tactics, override ``assign_roles``, customize
``select_chaser`` or ``kick_target``, or register a new role after
``super().__init__(kit)``.
"""

from __future__ import annotations

import math
import time
from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import TYPE_CHECKING

from ..soccer_framework import PlayContext, ReadySlot, RobotCommand
from ..runtime import SoccerKit

if TYPE_CHECKING:
    from ..soccer_framework import BallState
    from .role import RoleRegistry, RoleStrategy


# Default role labels kept as constants for ``assign_roles``; they no longer restrict the string set.
ROLE_CHASER = "chaser"
ROLE_SUPPORTER = "supporter"
ROLE_GOALKEEPER = "goalkeeper"
ROLE_NONE = "none"


@dataclass(frozen=True)
class RoleAssignment:
    """Snapshot of this tick's dynamic role assignment.

    The storage is ``by_player: Mapping[int, str]`` where keys are player IDs
    and values are role labels. ``role_of`` returns :data:`ROLE_NONE` when absent.

    Construction is direct: ``RoleAssignment({1: "chaser", 2: "supporter"})``.

    Use :meth:`players_of` for reverse lookup by role; specialized attributes
    such as chaser/supporters are intentionally not provided.

    ``chaser_id`` / ``goalkeeper_id`` snapshot this tick's assigned chaser and
    goalkeeper player IDs so downstream roles (e.g. the supporter) can anchor to
    the **assigned** chaser instead of re-deriving it by scanning teammates.
    When the goalkeeper is the assigned chaser (ball in the defensive area and
    keeper is closest), ``chaser_id == goalkeeper_id`` — supporters detect this
    and switch to fixed-field-reference danger-zone positioning.
    """

    by_player: Mapping[int, str] = field(default_factory=dict)
    chaser_id: int | None = None
    goalkeeper_id: int | None = None

    def __post_init__(self) -> None:
        # Freeze as a read-only view so external by_player edits cannot change ``role_of`` behavior.
        object.__setattr__(self, "by_player", MappingProxyType(dict(self.by_player)))

    def role_of(self, player_id: int) -> str:
        return self.by_player.get(player_id, ROLE_NONE)

    def players_of(self, name: str) -> tuple[int, ...]:
        return tuple(pid for pid, role in self.by_player.items() if role == name)


class Playbook:
    """Entry point for all PLAY-stage decisions competitors may override.

    Leaf nodes hold both :class:`SoccerKit` for tools and :class:`Playbook`
    for decisions. In the PLAY subtree, ``AssignRoles`` writes
    :meth:`assign_roles` output, per-player branches choose by
    :class:`RoleAssignment` plus ``role_registry``, kick leaves call
    :meth:`kick_target`, hold-style roles own their targets, and
    ``WaitForBall`` calls :meth:`waiting_command`.

    Subclasses add or remove roles in ``__init__`` with :meth:`register_role` and
    override :meth:`assign_roles` to choose this tick's assignment.
    """

    def __init__(self, kit: SoccerKit):
        # Delayed import breaks the role -> kit.blackboard -> kit -> play -> playbook cycle.
        from .role import RoleRegistry

        self.kit = kit
        self._registry = RoleRegistry()

    # Role registry

    def register_role(self, role: RoleStrategy) -> "Playbook":
        """Register a role on this Playbook and return self for chaining.

        Subclasses call this in ``__init__`` after ``super().__init__(kit)``.
        Registration order determines PLAY Selector branch priority.
        """

        self._registry.register(role)
        return self

    @property
    def role_registry(self) -> RoleRegistry:
        return self._registry

    # Role assignment, the core strategy node

    def assign_roles(
        self,
        context: PlayContext,
    ) -> RoleAssignment:
        """Return one :class:`RoleAssignment` per tick; subclasses decide the actual assignment."""

        raise NotImplementedError

    # Cross-role cooperative targets

    def waiting_command(
        self,
        player_id: int,
        context: PlayContext,
    ) -> RobotCommand:
        """Fallback command when no role is assigned.

        The default is a stop tagged by ReadySlot. Competitors can override this
        for custom idle positioning; the node has already cleared :class:`KickHysteresis`.
        """

        slot = self.kit.config.ready_slot_for_player(player_id)
        return RobotCommand.stop(f"{slot.value} waiting for ball")


# ----------------------------------------------------------------------
# Default implementation: fixed ReadySlot starts plus PLAY dynamic roles
# ----------------------------------------------------------------------


class DefaultPlaybook(Playbook):
    """Default SoccerSim playbook: chaser/supporter/goalkeeper dynamic roles plus Targeting scores.

    Subclasses can selectively override one method, for example:

    .. code-block:: python

    class AggressivePlaybook(DefaultPlaybook):
    def assign_roles(self, context):
    base = super().assign_roles(context)
    Move more players to supporters when trailing.
    """

    def __init__(self, kit: SoccerKit):
        super().__init__(kit)
        # Explicitly register default PLAY dynamic roles; competitor subclasses can
        # call register_role(...) after super().__init__(kit). DefenderRole is reserved for explicit custom Playbook registration.
        from .default_roles import (
            ChaserRole,
            GoalkeeperRole,
            SupporterRole,
        )

        self.register_role(ChaserRole())
        self.register_role(SupporterRole())
        self.register_role(GoalkeeperRole())
        self._keeper_could_challenge = False
        self._keeper_clear_locked_until = 0.0
        self._last_chaser_id: int | None = None
        self._last_game_state: object = None
        self._last_perf_log_at: float = 0.0
        self._chaser_lock_until: float = 0.0

    def assign_roles(self, context: PlayContext) -> RoleAssignment:
        chaser_id = self.select_chaser(context)
        goalkeeper_id = self._configured_goalkeeper()
        ball = context.known_ball

        mapping: dict[int, str] = {}
        for player_id in self.kit.config.player_ids:
            if player_id == goalkeeper_id:
                mapping[player_id] = ROLE_GOALKEEPER
            elif player_id == chaser_id:
                mapping[player_id] = ROLE_CHASER
            else:
                mapping[player_id] = ROLE_SUPPORTER

        now = time.time()
        if now - self._last_perf_log_at >= 2.0:
            self._last_perf_log_at = now
            self._log_performance(context, mapping, chaser_id, goalkeeper_id)

        return RoleAssignment(
            mapping, chaser_id=chaser_id, goalkeeper_id=goalkeeper_id,
        )

    # Internals

    def _configured_goalkeeper(self) -> int | None:
        return self.kit.config.goalkeeper_player_id()

    def _chaser_lock_defensive_x(self) -> float:
        """Defensive-zone x boundary for the chaser lock ladder (issue 3.1).

        Decoupled from the goalkeeper challenge area; driven by
        ``chaser_lock_defensive_x_ratio``. Own-side, negative x.
        """
        config = self.kit.config
        return -config.field_length * config.strategy.chaser_lock_defensive_x_ratio

    def _chaser_lock_duration(self, ball: BallState) -> float:
        """Dynamic chaser-lock duration (s) based on ball danger level.

        Three tiers, from most to least dangerous:
        - near own goal line (``< own_goal_x + 1.5``) -> 3.0 s
        - inside defensive zone (``< defensive_x``)    -> 2.0 s
        - otherwise                                    -> 1.5 s (config: chaser_lock_midfield_duration_s)
        """
        config = self.kit.config
        own_goal_x = -config.field_length / 2.0  # -7.0
        if ball.x < own_goal_x + 1.5:
            return 3.0
        if ball.x < self._chaser_lock_defensive_x():
            return 2.0
        return config.strategy.chaser_lock_midfield_duration_s

    def select_chaser(self, context: PlayContext) -> int:
        """Select this tick's chaser from our team.

        Decision priority:
        1. ReadySlot eligibility: keeper only joins dangerous balls, side only challenges when suitable.
        2. ``ball_claim_score``: cost based on distance to ball plus ReadySlot preference.
        3. Lowest player ID wins ties for predictable debugging.

        If no role is suitable, fall back to the smallest configured player ID so
        "nobody chases" does not become an extra ``None`` state. Override this method
        or call another score from :meth:`assign_roles` to change chase strategy.
        """
        config = self.kit.config
        targeting = self.kit.targeting
        ball = context.known_ball

        opponent_poses = tuple(
            opp.pose
            for opp in context.opponents.values()
            if opp.pose is not None
        )

        candidates: list[int] = []
        scored: list[tuple[float, int]] = []
        for player_id in config.player_ids:
            slot = config.ready_slot_for_player(player_id)
            if not self._slot_can_challenge(slot, context):
                continue
            candidates.append(player_id)
            robot = context.teammates.get(player_id)
            if robot is None or robot.pose is None:
                continue
            base_score = targeting.ball_claim_score(slot, robot.pose, ball)
            penalty = targeting.opponent_approach_penalty(robot.pose, ball, opponent_poses)
            scored.append((base_score + penalty, player_id))

        if not candidates:
            chaser_id = min(config.player_ids)
        elif not scored:
            chaser_id = min(candidates)
        else:
            tie_margin = config.strategy.teammate_challenge_tie_margin_m
            ranked = sorted(scored, key=lambda item: item[0])
            best_score = ranked[0][0]
            tied_ids = [
                player_id for score, player_id in ranked if score <= best_score + tie_margin
            ]
            chaser_id = min(tied_ids)

            # Dynamic lock duration based on ball danger level
            lock_duration = self._chaser_lock_duration(ball)

            # Refresh lock every tick while ball is in defensive area,
            # so the chaser never gets switched mid-approach.
            now = time.time()
            if ball.x < self._chaser_lock_defensive_x() and self._last_chaser_id is not None:
                min_lock = now + lock_duration
                if self._chaser_lock_until < min_lock:
                    self._chaser_lock_until = min_lock

            # Chaser switching lock: keep current chaser briefly to prevent ping-pong
            if (self._last_chaser_id is not None
                    and now < self._chaser_lock_until
                    and chaser_id != self._last_chaser_id):
                robot = context.teammates.get(self._last_chaser_id)
                if robot is not None and robot.pose is not None:
                    slot = config.ready_slot_for_player(self._last_chaser_id)
                    current_base = targeting.ball_claim_score(slot, robot.pose, ball)
                    current_penalty = targeting.opponent_approach_penalty(robot.pose, ball, opponent_poses)
                    current_score = current_base + current_penalty
                    if current_score < best_score + config.strategy.chaser_lock_score_margin_m:
                        chaser_id = self._last_chaser_id

        if chaser_id != self._last_chaser_id:
            now = time.time()
            lock_duration = self._chaser_lock_duration(ball)
            self._chaser_lock_until = now + lock_duration
            self._last_chaser_id = chaser_id
            logger = self.kit.logger
            if logger is not None:
                candidate_info = {}
                for score, pid in scored:
                    candidate_info[str(pid)] = {
                        "score": round(score, 3),
                        "slot": config.ready_slot_for_player(pid).value,
                    }
                logger.info(
                    f"chaser selected player={chaser_id} "
                    f"ball=({ball.x:.3f},{ball.y:.3f}) "
                    f"candidates={len(candidate_info)}",
                    event="chaser_selected",
                    chaser_id=chaser_id,
                    ball_x=round(ball.x, 3),
                    ball_y=round(ball.y, 3),
                    candidates=candidate_info,
                    opponents_tracked=len(opponent_poses),
                )

        return chaser_id

    def _slot_can_challenge(
        self,
        slot: ReadySlot,
        context: PlayContext,
    ) -> bool:
        targeting = self.kit.targeting
        ball = context.known_ball
        if slot == ReadySlot.KEEPER:
            game = context.known_game
            if self._last_game_state is not None and game.state != self._last_game_state:
                self._keeper_could_challenge = False
                self._keeper_clear_locked_until = 0.0
            self._last_game_state = game.state

            # Time-hold: stay in clear for at least goalkeeper_clear_hold_sec
            # after entering it, even if the ball briefly leaves the area.
            now = time.monotonic()
            if now < self._keeper_clear_locked_until:
                return True

            raw = targeting.ball_in_own_defensive_area(ball)
            hyst = self.kit.config.strategy.goalkeeper_challenge_hysteresis_m

            if self._keeper_could_challenge and hyst > 0.0:
                area_x, area_y = self.kit.targeting.goalkeeper_defensive_area()
                can = ball.x < area_x + hyst and abs(ball.y) <= area_y + hyst
            else:
                can = raw

            if can != self._keeper_could_challenge:
                self._keeper_could_challenge = can
                if can:
                    # Arm the time-hold so the challenge decision is sticky.
                    self._keeper_clear_locked_until = (
                        now + self.kit.config.strategy.goalkeeper_clear_hold_sec
                    )
                logger = self.kit.logger
                if logger is not None:
                    reason = "ball in defensive area" if can else "ball outside defensive area"
                    logger.info(
                        f"GK can{'not' if not can else ''} challenge: {reason} "
                        f"ball=({ball.x:.3f},{ball.y:.3f})",
                        event="goalkeeper_challenge",
                        can_challenge=can,
                        ball_x=round(ball.x, 3), ball_y=round(ball.y, 3),
                    )
            return can
        if slot == ReadySlot.SIDE:
            return targeting.side_should_challenge(context)
        return True

    def _log_performance(
        self,
        context: PlayContext,
        mapping: dict[int, str],
        chaser_id: int | None,
        goalkeeper_id: int | None,
    ) -> None:
        logger = self.kit.logger
        if logger is None:
            return
        ball = context.known_ball
        players: list[dict[str, object]] = []
        parts: list[str] = [f"perf ball=({ball.x:.2f},{ball.y:.2f})"]
        for player_id, role in mapping.items():
            robot = context.teammates.get(player_id)
            if robot is None or robot.pose is None:
                continue
            dist = math.hypot(ball.x - robot.pose.x, ball.y - robot.pose.y)
            players.append({
                "id": player_id,
                "role": role,
                "ball_dist": round(dist, 2),
                "x": round(robot.pose.x, 2),
                "y": round(robot.pose.y, 2),
                "in_attack": robot.pose.x > 0,
            })
            parts.append(f"p{player_id}:{role} d={dist:.2f}")

        field_len = self.kit.config.field_length
        zone = (
            "danger" if ball.x < -field_len * 0.25 else
            "defensive" if ball.x < 0 else
            "midfield" if ball.x < field_len * 0.25 else
            "attack"
        )

        closest = min(players, key=lambda p: p["ball_dist"]) if players else None

        xs = [
            robot.pose.x
            for robot in context.teammates.values()
            if robot is not None and robot.pose is not None
        ]
        centroid_x = round(sum(xs) / len(xs), 2) if xs else 0.0

        game = context.game
        our_score = -1
        opp_score = -1
        if game is not None:
            our = game.get_team_state(self.kit.config.team_id)
            opp = game.get_team_state(self.kit.config.opponent_team_id())
            if our is not None:
                our_score = our.score
            if opp is not None:
                opp_score = opp.score

        parts.append(
            f"zone={zone}"
            + (f" close={closest['id']} d={closest['ball_dist']:.2f}" if closest else "")
            + f" cx={centroid_x} score={our_score}:{opp_score}"
        )

        logger.info(
            " ".join(parts),
            event="performance_summary",
            ball_x=round(ball.x, 2),
            ball_y=round(ball.y, 2),
            ball_zone=zone,
            players=players,
            closest_player=closest,
            team_centroid_x=centroid_x,
            our_score=our_score,
            opp_score=opp_score,
        )
