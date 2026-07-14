"""Support-position targets plus teammate-spacing pushout.

SafetyGuards ensure PLAY support targets only run with fresh ball and
GameController data; the supporter positions on the ball-to-own-goal-center
line (max 3 m behind the ball) and pushes away from teammates to avoid stacking.
"""

from __future__ import annotations

import math

from ...soccer_framework import (
    BallState,
    Pose2D,
    SoccerConfig,
    PlayContext,
)
from ..geometry import TeamFieldFrame
from .attack import PlayerAllowed


__all__ = ["support_target"]


def support_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    player_id: int,
    context: PlayContext,
    is_player_allowed: PlayerAllowed,
) -> tuple[Pose2D, bool]:
    """Compute this tick's supporter target Pose2D.

    Positions the supporter behind the **chaser** (not the ball) at a dynamic
    distance of 1–4 m, offset laterally by 10°–45° to form a triangle.
    Always faces the ball via ``theta = face_ball_theta`` so the supporter can
    react instantly when the ball passes the chaser.

    Distance rule (relative to chaser):
      current < 1 m   → target at 1 m  (back up, strafe mode keeps facing ball)
      current > 2.8 m → target at 2.8 m  (close in)
      otherwise       → keep distance, only lateral strafe to adjust triangle angle

    Falls back to ball→own-goal line positioning when no chaser or own pose is
    available.

    Returns (target, was_pushed).
    """

    ball = context.known_ball
    game = context.known_game

    # Find chaser: non-GK teammate closest to the ball
    gk_id = config.goalkeeper_player_id()
    chaser_pose = None
    chaser_dist = float("inf")
    for tid, trobot in context.teammates.items():
        if (
            tid == player_id
            or trobot.pose is None
            or not is_player_allowed(game, tid)
        ):
            continue
        if tid == gk_id:
            continue
        dist = math.hypot(ball.x - trobot.pose.x, ball.y - trobot.pose.y)
        if dist < chaser_dist:
            chaser_dist = dist
            chaser_pose = trobot.pose

    own_robot = context.teammates.get(player_id)

    # Need both chaser pose and own pose for chaser-relative positioning.
    if chaser_pose is not None and own_robot is not None and own_robot.pose is not None:
        tx, ty = _chaser_relative_target(
            config, ball, chaser_pose, own_robot.pose, player_id,
        )
    else:
        # Fallback: ball → own-goal line at 2.5 m behind ball
        tx, ty = _goal_line_fallback(config, field, ball)

    target = field.clamp_inside_field(
        Pose2D(tx, ty, field.face_ball_theta(tx, ty, ball))
    )
    return _spaced_support_target(
        config,
        field,
        player_id,
        context,
        target,
        is_player_allowed,
    )


def _chaser_relative_target(
    config: SoccerConfig,
    ball: BallState,
    chaser_pose: Pose2D,
    own_pose: Pose2D,
    player_id: int,
) -> tuple[float, float]:
    """Compute supporter position behind the chaser with distance clamping.

    Direction: blends stable ball→own-goal with ball→chaser, weighted by how
    far the chaser is from the ball.  When the chaser is near the ball the
    goal direction dominates (stable); when the chaser is farther away the
    chaser direction takes over (accurate).

    Distance:  clamped to [1, 2.8] m from the chaser based on current separation.
    Angle:     10°–45° lateral offset, scaled down near sidelines to avoid
               clamping the target into the field corner.
    """

    # ── P1: stable behind-direction = goal-direction / chaser-direction blend ──
    goal_dx = -config.field_length / 2.0 - ball.x
    goal_dy = -ball.y
    goal_len = math.hypot(goal_dx, goal_dy)
    goal_ux = goal_dx / goal_len
    goal_uy = goal_dy / goal_len

    bc_dx = chaser_pose.x - ball.x
    bc_dy = chaser_pose.y - ball.y
    bc_len = math.hypot(bc_dx, bc_dy)

    if bc_len < 1e-6:
        bx, by = goal_ux, goal_uy
    else:
        bc_ux, bc_uy = bc_dx / bc_len, bc_dy / bc_len
        # blend: 0.0 at bc_len≤0.3 (goal-only), 1.0 at bc_len≥0.8 (chaser-only)
        blend = max(0.0, min(1.0, (bc_len - 0.3) / 0.5))
        bx = goal_ux + (bc_ux - goal_ux) * blend
        by = goal_uy + (bc_uy - goal_uy) * blend
        bl = math.hypot(bx, by)
        if bl > 1e-6:
            bx, by = bx / bl, by / bl

    # ── Distance: clamped to [1, 2.8] m from chaser ──
    sc_dist = math.hypot(
        own_pose.x - chaser_pose.x, own_pose.y - chaser_pose.y,
    )
    if sc_dist < 1.0:
        desired_dist = 1.0
    elif sc_dist > 2.8:
        desired_dist = 2.8
    else:
        desired_dist = sc_dist

    # ── P2: triangle angle scaled down near sideline to avoid corner clamp ──
    t = max(0.0, min(1.0,
        (ball.x + config.field_length / 2.0) / config.field_length,
    ))
    base_deg = 10.0 + t * 35.0
    dist_to_sideline = config.field_width / 2.0 - abs(ball.y)
    angle_scale = max(0.0, min(1.0, dist_to_sideline / 1.5))
    angle_rad = math.radians(10.0 + (base_deg - 10.0) * angle_scale)

    # Side alternates by player_id parity.
    side = 1.0 if player_id % 2 == 0 else -1.0
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    dir_x = bx * cos_a - by * sin_a * side
    dir_y = bx * sin_a * side + by * cos_a

    tx = chaser_pose.x + desired_dist * dir_x
    ty = chaser_pose.y + desired_dist * dir_y

    # ── P3: back off along direction if target would leave the field ──
    # Avoid clamp_inside_field jumping the target to a far-away boundary
    # corner; instead shorten the distance so the target slides along the
    # boundary naturally.
    half_x = config.field_length / 2.0 - 0.25
    half_y = config.field_width / 2.0 - 0.25
    if not (-half_x <= tx <= half_x and -half_y <= ty <= half_y):
        scale = 1.0
        cx, cy = chaser_pose.x, chaser_pose.y
        if abs(tx - cx) > 1e-6:
            limit = half_x if tx > cx else -half_x
            scale = min(scale, (limit - cx) / (tx - cx))
        if abs(ty - cy) > 1e-6:
            limit = half_y if ty > cy else -half_y
            scale = min(scale, (limit - cy) / (ty - cy))
        if scale > 0.0 and scale * desired_dist >= 0.5:
            tx = cx + (tx - cx) * scale
            ty = cy + (ty - cy) * scale

    return tx, ty


def _goal_line_fallback(
    config: SoccerConfig,
    field: TeamFieldFrame,
    ball: BallState,
) -> tuple[float, float]:
    """Ball → own-goal line at 2.5 m behind the ball (used when no chaser)."""

    GK = field.own_goal_x()
    dx = ball.x - GK
    dy = ball.y
    dist = math.hypot(dx, dy)
    max_dist = 2.5
    ratio = max(dist - max_dist, 0.0) / max(dist, 0.01)
    return GK + dx * ratio, dy * ratio


# Teammate spacing pushout


def _spaced_support_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    player_id: int,
    context: PlayContext,
    target: Pose2D,
    is_player_allowed: PlayerAllowed,
) -> tuple[Pose2D, bool]:
    """If target is closer than min_spacing to the nearest teammate, push it along "teammate -> target" out to ``min_spacing``.

    Steps:
    1. Find the nearest legal teammate.
    2. If distance is large enough, do nothing.
    3. Otherwise scale the "teammate -> target" unit vector to min_spacing.
    4. Clamp inside the field and finally face the ball.

    Degenerate case: when target almost overlaps the teammate, no direction can
    be scaled, so fall back to ``lane_sign`` based on which side of the ball target
    is on; if target is exactly on the ball, split by player_id parity.

    In extreme corners with teammate pressure, clamping can make the final target
    slightly closer than min_spacing. With at most three teammates this is rare; if
    strict final distance is needed, iterate once more after clamping.

    Returns (adjusted_target, was_pushed).
    """

    min_spacing = config.strategy.support_min_spacing_m
    if min_spacing <= 0.0:
        return target, False

    ball = context.known_ball
    game = context.known_game
    teammate_poses = tuple(
        robot.pose
        for teammate_id, robot in context.teammates.items()
        if teammate_id != player_id
        and robot.pose is not None
        and is_player_allowed(game, teammate_id)
    )
    if not teammate_poses:
        return target, False

    closest = min(
        teammate_poses,
        key=lambda pose: math.hypot(pose.x - target.x, pose.y - target.y),
    )
    dx = target.x - closest.x
    dy = target.y - closest.y
    distance = math.hypot(dx, dy)
    if distance >= min_spacing:
        return target, False

    if distance <= 1e-6:
        lane_sign = 1.0 if target.y >= ball.y else -1.0
        if abs(target.y - ball.y) < 1e-6:
            lane_sign = 1.0 if player_id % 2 == 0 else -1.0
        dx, dy = 0.0, lane_sign
        distance = 1.0

    scale = min_spacing / distance
    pushed = field.clamp_inside_field(
        Pose2D(
            closest.x + dx * scale,
            closest.y + dy * scale,
            target.theta,
        )
    )
    return Pose2D(
        pushed.x,
        pushed.y,
        field.face_ball_theta(pushed.x, pushed.y, ball),
    ), True
