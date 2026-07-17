"""Stateless field predicates and player scoring functions.

This module only performs pure calculations from ``BallState`` / ``PlayContext``
plus geometry thresholds to bool/float results. It does not create :class:`Pose2D`
or dispatch commands; other targeting modules call these predicates before higher-level decisions.
"""

from __future__ import annotations

import math

from ...soccer_framework import (
    BallState,
    Pose2D,
    ReadySlot,
    SoccerConfig,
    PlayContext,
)


__all__ = [
    "ball_beyond_goal_line",
    "ball_beyond_own_goal_line",
    "ball_claim_score",
    "opponent_approach_penalty",
    "ball_in_own_defensive_area",
    "ball_is_in_midfield_or_own_half",
    "ball_near_sideline",
    "goalkeeper_defensive_area",
    "pose_for_slot",
    "side_should_challenge",
    "sideline_sign",
]


# Field-geometry predicates


def goalkeeper_defensive_area(config: SoccerConfig) -> tuple[float, float]:
    """Return (area_x, area_y) boundaries of the goalkeeper's defensive challenge area."""
    area_x = -config.field_length * config.strategy.goalkeeper_challenge_area_x_ratio
    area_y = min(
        config.field_width / 2.0 - 0.35,
        config.strategy.goalkeeper_challenge_area_y,
    )
    return area_x, area_y


def ball_in_own_defensive_area(config: SoccerConfig, ball: BallState) -> bool:
    """Whether the ball is in our dangerous area where the goalkeeper should clear it."""
    area_x, area_y = goalkeeper_defensive_area(config)
    return ball.x < area_x and abs(ball.y) <= area_y


def ball_beyond_goal_line(config: SoccerConfig, ball: BallState) -> bool:
    """Whether the ball crossed either goal line, ours or the opponent's."""

    half_length = config.field_length / 2.0
    margin = config.strategy.goal_line_recovery_margin_m
    return abs(ball.x) > half_length + margin


def ball_beyond_own_goal_line(config: SoccerConfig, ball: BallState) -> bool:
    """Whether the ball crossed our goal line in the ``-x`` direction."""

    half_length = config.field_length / 2.0
    margin = config.strategy.goal_line_recovery_margin_m
    return ball.x < -half_length - margin


def ball_near_sideline(config: SoccerConfig, ball: BallState) -> bool:
    """Whether the ball is close enough to the sideline to trigger sideline recovery."""

    sideline_y = config.field_width / 2.0
    return (
        abs(ball.y)
        >= sideline_y - config.strategy.sideline_recovery_margin_m
    )


def ball_is_in_midfield_or_own_half(config: SoccerConfig, ball: BallState) -> bool:
    """Whether the ball is around midfield or our half; SIDE uses this to decide whether to challenge."""

    return ball.x < config.field_length * config.strategy.midfield_boundary_x_ratio


def sideline_sign(y: float) -> float:
    """Return +1 for the upper side of the field and -1 for the lower side."""

    return 1.0 if y >= 0.0 else -1.0


# Player scoring


def ball_claim_score(
    config: SoccerConfig,
    slot: ReadySlot,
    pose: Pose2D,
    ball: BallState,
) -> float:
    """Estimate the cost for a player to claim the current ball; lower score is better for chaser.

    KEEPER gets a high field-length cost unless the ball is dangerous, pushing it
    to the end. CENTER is slightly cheaper than SIDE to prefer central challenges.
    """

    distance = math.hypot(ball.x - pose.x, ball.y - pose.y)
    if slot == ReadySlot.KEEPER:
        if ball_in_own_defensive_area(config, ball):
            return distance - 0.75
        return distance + config.field_length
    if slot == ReadySlot.CENTER:
        return distance - 0.20
    return distance - 0.10


def opponent_approach_penalty(
    config: SoccerConfig,
    teammate_pose: Pose2D,
    ball: BallState,
    opponent_poses: tuple[Pose2D, ...],
) -> float:
    """Opponent-interference penalty (meters-equivalent) for a chaser candidate.

    Lower = better. Returns 0.0 when no opponents are tracked (degrades to the
    pre-6.1 behavior where ball_claim_score alone decided the chaser).

    Three components, summed and capped (per Oracle review of issue 6.1):
    1. Lane block: continuous projection onto teammate->ball segment, radius
       chaser_lane_block_radius_m. A DEDICATED implementation is used here instead
       of reusing lane_clear_score, because lane_clear_score applies
       max(pass_lane_clearance=0.75, radius) which is too wide for approach paths.
    2. Ball contest: radial distance to ball, radius chaser_opponent_contest_radius_m.
    3. Marking: radial distance to teammate, radius chaser_marking_radius_m.

    Aggregation uses sum (not max) so that multi-opponent pressure accumulates;
    the total is capped at chaser_max_interference_penalty_m.
    """
    if not opponent_poses:
        return 0.0
    penalty = 0.0
    # 1. Lane block (continuous, segment-clamped, dedicated projection)
    seg_dx = ball.x - teammate_pose.x
    seg_dy = ball.y - teammate_pose.y
    seg_len = math.hypot(seg_dx, seg_dy)
    if seg_len > 1e-6:
        dir_x, dir_y = seg_dx / seg_len, seg_dy / seg_len
        left_x, left_y = -dir_y, dir_x
        clearance = config.strategy.chaser_lane_block_radius_m
        lane_score = 1.0  # 1.0 = fully clear, 0.0 = fully blocked
        for opp in opponent_poses:
            rel_x = opp.x - teammate_pose.x
            rel_y = opp.y - teammate_pose.y
            along = rel_x * dir_x + rel_y * dir_y
            # Ignore opponents outside the segment (behind teammate or beyond ball)
            if along <= 0.0 or along >= seg_len:
                continue
            lateral = abs(rel_x * left_x + rel_y * left_y)
            if lateral < clearance:
                lane_score = min(lane_score, lateral / clearance)
        penalty += (1.0 - lane_score) * config.strategy.chaser_lane_penalty_weight
    # 2. Ball contest (opponent near the ball challenges possession)
    contest_radius = config.strategy.chaser_opponent_contest_radius_m
    for opp in opponent_poses:
        d_ball = math.hypot(opp.x - ball.x, opp.y - ball.y)
        if d_ball < contest_radius:
            penalty += config.strategy.chaser_contest_weight * (1.0 - d_ball / contest_radius)
    # 3. Marking (opponent tight on the teammate restricts their approach)
    marking_radius = config.strategy.chaser_marking_radius_m
    for opp in opponent_poses:
        d_tm = math.hypot(opp.x - teammate_pose.x, opp.y - teammate_pose.y)
        if d_tm < marking_radius:
            penalty += config.strategy.chaser_marking_weight * (1.0 - d_tm / marking_radius)
    return min(penalty, config.strategy.chaser_max_interference_penalty_m)


def pose_for_slot(
    config: SoccerConfig,
    context: PlayContext,
    slot: ReadySlot,
) -> Pose2D | None:
    """Find the teammate pose currently assigned to a ReadySlot; return ``None`` if missing."""

    for player_id, robot in context.teammates.items():
        if config.ready_slot_for_player(player_id) == slot:
            return robot.pose
    return None


def side_should_challenge(
    config: SoccerConfig,
    context: PlayContext,
) -> bool:
    """Whether the SIDE slot should challenge: always in midfield/own half, and in attack only when clearly closer than CENTER."""

    if context.ball is None:
        return False
    ball = context.known_ball
    if ball_is_in_midfield_or_own_half(config, ball):
        return True
    center = pose_for_slot(config, context, ReadySlot.CENTER)
    side = pose_for_slot(config, context, ReadySlot.SIDE)
    if center is None or side is None:
        return False
    center_dist = math.hypot(ball.x - center.x, ball.y - center.y)
    side_dist = math.hypot(ball.x - side.x, ball.y - side.y)
    return side_dist + 0.20 < center_dist
