"""READY-stage positioning: base targets for three ReadySlots plus SetPlay variants.

Pure model extracted from :class:`SoccerKit`; it depends only on
:class:`SoccerConfig` and :class:`TeamFieldFrame`, owns no cross-tick state, and is
held by :class:`SoccerKit`.
"""

from __future__ import annotations

import math

from ..soccer_framework import (
    BallState,
    GameControlState,
    Pose2D,
    ReadySlot,
    SetPlay,
    SoccerConfig,
)
from .geometry import TeamFieldFrame, clamp, lerp, smoothstep


class ReadyStance:
    """READY-stage target-position calculation.

    Three pieces of logic:

    :meth:`base_ready_target`: base positions for CENTER, SIDE, and KEEPER.
    :meth:`ready_target_for`: final target from current SetPlay and ball position,
    including own restart, opponent restart, or base target.
    :meth:`ready_targets_for`: batch assignment for all available players based on
    kickoff type and player count (6 scenarios).
    :meth:`goalkeeper_guard_target`: goalkeeper guard-position formula.
    """

    def __init__(self, config: SoccerConfig, field: TeamFieldFrame):
        self.config = config
        self.field = field

    def base_ready_target(
        self,
        slot: ReadySlot,
        own_restart: bool,
    ) -> Pose2D:
        """Base target for the three ReadySlots: center, side, and keeper.

        ``own_restart`` means the restart belongs to us. Own restarts push the
        attacker toward the center circle; opponent restarts pull back for safety.
        """
        field_length = self.config.field_length
        field_width = self.config.field_width
        circle_radius = self.config.center_circle_radius
        goal_area_length = self.config.goal_area_length

        goal_x = self.field.own_goal_x() + goal_area_length + 0.25
        side_y = min(field_width / 2.0 - 0.45, max(0.9, field_width * 0.30))
        attack_x = -max(
            circle_radius * (0.95 if own_restart else 1.6),
            field_length * (0.12 if own_restart else 0.20),
        )
        attack_line_x = self.field.own_half_x(attack_x, margin=0.15)

        if slot == ReadySlot.CENTER:
            return Pose2D(
                x=attack_line_x,
                y=0.0,
                theta=self.field.attack_theta(),
            )
        if slot == ReadySlot.SIDE:
            return Pose2D(
                x=attack_line_x,
                y=side_y,
                theta=self.field.attack_theta(),
            )
        return Pose2D(x=goal_x, y=0.0, theta=self.field.attack_theta())

    def ready_target_for(
        self,
        slot: ReadySlot,
        game: GameControlState,
        ball: BallState | None,
    ) -> Pose2D:
        """Compute READY positioning from the current SetPlay and ball position.

        No SetPlay or no ball: use base target.
        Own restart: stand close to the ball, ready to restart.
        Opponent restart: avoid the configured area around the ball.
        """
        own_restart = game.is_restart_for_team(self.config.team_id)
        base_target = self.base_ready_target(slot, own_restart)
        if game.set_play == SetPlay.NONE or ball is None:
            return base_target
        if own_restart:
            return self._own_set_play_ready_target(slot, ball, base_target)
        return self.field.avoid_ball_target(base_target, ball)

    # (is_own_kickoff, available_count) → {ReadySlot: Pose2D}
    # Positions are mapped by slot name so READY→PLAYING slot alignment is preserved.
    _SCENARIO_POSITIONS: dict[tuple[bool, int], dict[ReadySlot, Pose2D]] = {
        (True, 3): {
            ReadySlot.CENTER: Pose2D(-0.75, -0.75, 0.785),
            ReadySlot.SIDE:   Pose2D(-2.25, 1.35, 0.0),
            ReadySlot.KEEPER: Pose2D(-6.5, 0.0, 0.0),
        },
        (True, 2): {
            ReadySlot.CENTER: Pose2D(-0.75, -0.75, 0.785),
            ReadySlot.KEEPER: Pose2D(-6.5, 0.0, 0.0),
        },
        (True, 1): {
            ReadySlot.CENTER: Pose2D(-0.75, -0.75, 0.785),
        },
        (False, 3): {
            ReadySlot.CENTER: Pose2D(-2.80, -0.2, 0.0),
            ReadySlot.SIDE:   Pose2D(-4.5, 0.8, 0.0),
            ReadySlot.KEEPER: Pose2D(-6.0, -1.0, 0.0),
        },
        (False, 2): {
            ReadySlot.CENTER: Pose2D(-3.50, 0.2, 0.0),
            ReadySlot.KEEPER: Pose2D(-6.0, -1.0, 0.0),
        },
        (False, 1): {
            ReadySlot.CENTER: Pose2D(-4.0, 0.0, 0.0),
        },
    }

    _SLOT_PRIORITY: dict[ReadySlot, int] = {
        ReadySlot.CENTER: 0,
        ReadySlot.KEEPER: 1,
        ReadySlot.SIDE: 2,
    }

    def ready_targets_for(
        self,
        available_player_ids: list[int],
        is_own_kickoff: bool,
    ) -> dict[int, Pose2D]:
        """Batch-compute ready targets for all available players.

        Two-phase assignment:
        1. Direct slot match — if an available player's ReadySlot is in the
           scenario dict, assign that position directly (preserves alignment).
        2. Fill remaining — unmatched players fill vacant scenario positions,
           sorted by slot priority (center > keeper > side).

        Returns ``dict[player_id, Pose2D]`` containing only available players.
        """
        count = min(len(available_player_ids), 3)
        scenario = self._SCENARIO_POSITIONS.get((is_own_kickoff, count), {})

        result: dict[int, Pose2D] = {}
        unmatched: list[int] = []

        for pid in available_player_ids:
            slot = self.config.ready_slot_for_player(pid)
            if slot in scenario:
                result[pid] = scenario[slot]
            else:
                unmatched.append(pid)

        claimed_slots = {
            self.config.ready_slot_for_player(pid) for pid in result
        }
        vacant_slots = sorted(
            [s for s in scenario if s not in claimed_slots],
            key=lambda s: self._SLOT_PRIORITY.get(s, 99),
        )
        unmatched.sort(
            key=lambda pid: self._SLOT_PRIORITY.get(
                self.config.ready_slot_for_player(pid), 99
            ),
        )

        for pid, slot in zip(unmatched, vacant_slots):
            result[pid] = scenario[slot]

        return result

    def goalkeeper_guard_target(self, ball: BallState | None) -> Pose2D:
        """Goalkeeper guard formula; blends between goal-line deep and arc positioning.

        Arc radius and guard depth are dynamically scaled by ball distance from
        the goal line — close ball maximises angle coverage (full radius/depth),
        far ball pulls the keeper deeper (reduced radius) to protect against
        chips/long-range shots.
        """
        GK = self.field.own_goal_x()
        strat = self.config.strategy
        base_R = strat.goalkeeper_guard_arc_radius
        base_depth = strat.goalkeeper_guard_depth_m

        if ball is None:
            return Pose2D(GK + base_depth, 0.0, self.field.attack_theta())

        # Dynamic scale based on ball distance from our goal line.
        # At dist=0  (ball in goal) → scale=1.0 (full width/depth)
        # At dist=7  (half field)  → scale=0.6 (tight)
        ball_dist = math.hypot(ball.x - GK, ball.y)
        norm = min(ball_dist / (self.config.field_length * 0.5), 1.0)
        scale = 1.0 - 0.4 * norm  # 1.0 → 0.6
        R = base_R * scale
        depth = base_depth * (1.0 - 0.3 * norm)  # 1.3 → 0.91

        goal_line_x = GK + depth

        # Step 1 — arc intersection: ball-to-goal-center line ∩ circle(GK, R)
        dx = ball.x - GK
        dy = ball.y
        dist = math.hypot(dx, dy)
        t = min(R / max(dist, 0.01), 1.0)
        arc_x = GK + t * dx
        arc_y = t * dy

        # Step 2 — blend between goal line and arc based on lateral offset
        blend = smoothstep(0.7, 1.3, abs(arc_y))
        keeper_x = lerp(goal_line_x, arc_x, blend)
        keeper_x = max(keeper_x, goal_line_x)
        keeper_y = clamp(arc_y, -1.35, 1.35)

        theta = self.field.face_ball_theta(keeper_x, keeper_y, ball)

        target = Pose2D(keeper_x, keeper_y, theta)
        return self.field.clamp_from_goal_obstructions(target)

    def _own_set_play_ready_target(
        self,
        slot: ReadySlot,
        ball: BallState,
        base_target: Pose2D,
    ) -> Pose2D:
        """Own-restart close-ball positioning: center behind the ball, side diagonally behind for support."""
        if slot == ReadySlot.CENTER:
            return self.field.clamp_inside_field(
                Pose2D(
                    x=ball.x - 0.45,
                    y=ball.y,
                    theta=self.field.face_ball_theta(ball.x, ball.y, ball),
                )
            )
        if slot == ReadySlot.SIDE:
            y_offset = -1.1 if ball.y > 0.0 else 1.1
            return self.field.clamp_inside_field(
                Pose2D(
                    x=ball.x - 1.3,
                    y=ball.y + y_offset,
                    theta=self.field.face_ball_theta(ball.x, ball.y, ball),
                )
            )
        return base_target
