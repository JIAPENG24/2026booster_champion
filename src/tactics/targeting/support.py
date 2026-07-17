"""Support-position targets plus teammate/opponent spacing pushout.

SafetyGuards ensure PLAY support targets only run with fresh ball and
GameController data; the supporter positions behind the **chaser** (not the
ball) at a clamped distance and pushes away from teammates (anti-clustering)
and opponents (issue 8.1 — keep receiving room) to avoid stacking.
"""

from __future__ import annotations

import math

from ...soccer_framework import (
    BallState,
    Pose2D,
    SoccerConfig,
    PlayContext,
)
from ..geometry import TeamFieldFrame, normalize_angle
from .attack import PlayerAllowed


__all__ = ["support_target"]


def support_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    player_id: int,
    context: PlayContext,
    is_player_allowed: PlayerAllowed,
    *,
    chaser_id: int | None = None,
    danger_zone: bool = False,
) -> tuple[Pose2D, bool]:
    """Compute this tick's supporter target Pose2D.

    Positions the supporter behind the **chaser** (not the ball) at a dynamic
    distance of ``[support_min_distance_m, support_max_distance_m]`` (default
    1.0–2.2 m), offset laterally by 10°–45° to form a triangle.
    Always faces the ball via ``theta = face_ball_theta`` so the supporter can
    react instantly when the ball passes the chaser.

    Distance rule (relative to chaser):
      current < min  → target at min  (back up, strafe mode keeps facing ball)
      current > max  → target at max  (close in)
      otherwise      → hold current position when within the angular dead-zone
                       (anti-orbit), else adjust to the triangle angle.

    Chaser resolution (in priority order):
      1. ``danger_zone=True`` — the goalkeeper is the assigned chaser (ball in the
         defensive area, keeper closest), so NO outfielder has the chaser role.
         Use :func:`_danger_zone_support_target` (fixed field reference: one
         covers the goal line, one outlets upfield) so the two outfielders do not
         anchor to each other and drift to the corner flag.
      2. ``chaser_id`` given — anchor to the explicitly assigned outfield chaser
         (read from ``RoleAssignment``); this also fixes the case where the real
         chaser is not the nearest non-GK teammate (slot-bias selection).
      3. fallback — nearest legal non-GK teammate to the ball (legacy behaviour).

    Falls back to ball→own-goal line positioning when no chaser or own pose is
    available.

    Returns (target, was_pushed).
    """

    ball = context.known_ball
    game = context.known_game
    gk_id = config.goalkeeper_player_id()
    own_robot = context.teammates.get(player_id)

    # ── Danger zone: goalkeeper is the assigned chaser ──
    if danger_zone and own_robot is not None and own_robot.pose is not None:
        tx, ty = _danger_zone_support_target(config, field, ball, player_id, gk_id)
        target = field.clamp_inside_field(
            Pose2D(tx, ty, field.face_ball_theta(tx, ty, ball))
        )
        return _spaced_support_target(
            config, field, player_id, context, target, is_player_allowed,
        )

    # ── Resolve the chaser pose ──
    chaser_pose = None
    if chaser_id is not None:
        # Anchor to the explicitly assigned chaser when present and eligible.
        trobot = context.teammates.get(chaser_id)
        if (
            trobot is not None
            and trobot.pose is not None
            and chaser_id != player_id
            and chaser_id != gk_id
            and is_player_allowed(game, chaser_id)
        ):
            chaser_pose = trobot.pose

    if chaser_pose is None:
        # Fallback: nearest legal non-GK teammate to the ball.
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


def _danger_zone_support_target(
    config: SoccerConfig,
    field: TeamFieldFrame,
    ball: BallState,
    player_id: int,
    gk_id: int | None,
) -> tuple[float, float]:
    """Fixed-field-reference positioning when the goalkeeper is the chaser.

    The ball is in the defensive area and the keeper is the nearest player, so
    the keeper takes the chaser role and NO outfielder is the chaser. The two
    outfielders split by a stable index over the sorted outfield ids (robust to
    which player_id is the keeper):

      * **cover** (even index): block the direct shot — stand on the ball→own-goal
        center line, ``support_danger_cover_depth_m`` in front of the own goal, to
        guard the net while the keeper is committed to the ball.
      * **outlet** (odd index): stand upfield of the ball to receive a clearance,
        forward ``support_danger_outlet_forward_m`` and lateral
        ``±support_danger_outlet_lateral_m``.

    Both are field-relative (not teammate-relative), so they cannot drift with a
    moving or wrong chaser reference. Field clamping is applied by the caller.
    """

    strategy = config.strategy
    outfield_ids = [pid for pid in sorted(config.player_ids) if pid != gk_id]
    try:
        idx = outfield_ids.index(player_id)
    except ValueError:
        idx = player_id

    goal_x = field.own_goal_x()
    goal_y = 0.0

    if idx % 2 == 0:
        # Cover: on ball→goal-center line, cover_depth from the goal.
        dx = ball.x - goal_x
        dy = ball.y - goal_y
        dist = math.hypot(dx, dy)
        depth = strategy.support_danger_cover_depth_m
        t = depth / max(dist, 0.01)
        return goal_x + dx * t, goal_y + dy * t

    # Outlet: upfield + lateral by index parity (spread the two outlets apart).
    forward = strategy.support_danger_outlet_forward_m
    lateral = strategy.support_danger_outlet_lateral_m
    side = 1.0 if (idx // 2) % 2 == 0 else -1.0
    return ball.x + forward, ball.y + side * lateral


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

    Distance:  clamped to [support_min_distance_m, support_max_distance_m] from
               the chaser based on current separation.
    Angle:     10°–45° lateral offset, scaled down near sidelines to avoid
               clamping the target into the field corner.

    Stability (anti-orbit): once the supporter is inside the acceptable distance
    band AND within ``support_angle_hold_deadzone`` of the ideal angle, it holds
    its current position.  This stops the tangential "orbiting" that happens when
    the ideal angle is recomputed every frame from a moving ball/chaser — the
    supporter otherwise perpetually chases a target that slides along an arc and
    never settles.  Outside the band / dead-zone it fully repositions.
    """

    strategy = config.strategy
    min_dist = strategy.support_min_distance_m
    max_dist = strategy.support_max_distance_m
    hold_deadzone = strategy.support_angle_hold_deadzone

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

    # ── Distance: clamped to [min, max] from chaser ──
    sc_dx = own_pose.x - chaser_pose.x
    sc_dy = own_pose.y - chaser_pose.y
    sc_dist = math.hypot(sc_dx, sc_dy)
    if sc_dist < min_dist:
        desired_dist = min_dist
    elif sc_dist > max_dist:
        desired_dist = max_dist
    else:
        desired_dist = sc_dist

    # ── P2: triangle angle scaled down near sideline to avoid corner clamp ──
    # Reengage (P4): when far from the chaser, drop the lateral triangle offset
    # (angle=0 → straight behind the chaser) so the supporter beelines to close
    # the gap faster, instead of approaching at a wide triangle angle and lagging
    # behind fast attacks. The normal triangle resumes once within range.
    if sc_dist > strategy.support_reengage_distance_m:
        angle_rad = 0.0
    else:
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

    # ── Stability: dead-band hold to stop tangential orbiting ──
    # When the supporter already sits in the acceptable distance band and is
    # within an angular tolerance of the ideal direction, freeze at its current
    # position.  The only repositioning then comes from drift out of the band or
    # dead-zone, which tracks real chaser movement while ignoring ball wobble.
    if min_dist <= sc_dist <= max_dist and sc_dist > 1e-6:
        ideal_angle = math.atan2(ty - chaser_pose.y, tx - chaser_pose.x)
        cur_angle = math.atan2(sc_dy, sc_dx)
        if abs(normalize_angle(cur_angle - ideal_angle)) <= hold_deadzone:
            return own_pose.x, own_pose.y

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
    """Push the target out to legal spacing from teammates AND opponents.

    Two independent push-out passes are applied sequentially to ``target``:

    1. **Teammate spacing**: nearest legal teammate closer than
       ``support_min_spacing_m`` is pushed out along teammate→target to that
       radius (original anti-clustering behaviour).
    2. **Opponent avoidance** (issue 8.1): nearest opponent closer than
       ``support_opponent_avoid_radius_m`` is pushed out along
       opponent→target to that radius, so the supporter does not stand on top
       of an opponent and lose the ball instantly upon receiving it.

    Each pass is a best-effort single push; in extreme corners clamping can
    leave the final target slightly inside the radius.  The degenerate
    overlap case (target ~= obstacle) falls back to a lane sign derived from
    the ball side / player_id parity.

    Returns (adjusted_target, was_pushed).
    """

    strategy = config.strategy
    min_spacing = strategy.support_min_spacing_m
    opponent_radius = strategy.support_opponent_avoid_radius_m
    ball = context.known_ball
    game = context.known_game

    teammate_poses = tuple(
        robot.pose
        for teammate_id, robot in context.teammates.items()
        if teammate_id != player_id
        and robot.pose is not None
        and is_player_allowed(game, teammate_id)
    )
    opponent_poses = tuple(
        robot.pose
        for robot in context.opponents.values()
        if robot.pose is not None
    )

    was_pushed = False

    # Pass 1: nearest teammate spacing.
    if min_spacing > 0.0 and teammate_poses:
        closest_t = min(
            teammate_poses,
            key=lambda pose: math.hypot(pose.x - target.x, pose.y - target.y),
        )
        target, pushed = _push_out_from(
            field, ball, target, closest_t, min_spacing, player_id,
        )
        was_pushed = was_pushed or pushed

    # Pass 2: nearest opponent avoidance.
    if opponent_radius > 0.0 and opponent_poses:
        closest_o = min(
            opponent_poses,
            key=lambda pose: math.hypot(pose.x - target.x, pose.y - target.y),
        )
        target, pushed = _push_out_from(
            field, ball, target, closest_o, opponent_radius, player_id,
        )
        was_pushed = was_pushed or pushed

    return target, was_pushed


def _push_out_from(
    field: TeamFieldFrame,
    ball: BallState,
    target: Pose2D,
    obstacle: Pose2D,
    radius: float,
    player_id: int,
) -> tuple[Pose2D, bool]:
    """Push ``target`` out to ``radius`` from ``obstacle`` along obstacle→target.

    Returns the target unchanged when it is already at least ``radius`` away.
    Degenerate overlap (target ~= obstacle) resolves a direction from the ball
    side, falling back to player_id parity when the target is exactly on the
    ball.  Result is clamped inside the field and re-faced toward the ball.

    Returns (adjusted_target, was_pushed).
    """

    dx = target.x - obstacle.x
    dy = target.y - obstacle.y
    distance = math.hypot(dx, dy)
    if distance >= radius:
        return target, False

    if distance <= 1e-6:
        lane_sign = 1.0 if target.y >= ball.y else -1.0
        if abs(target.y - ball.y) < 1e-6:
            lane_sign = 1.0 if player_id % 2 == 0 else -1.0
        dx, dy = 0.0, lane_sign
        distance = 1.0

    scale = radius / distance
    pushed = field.clamp_inside_field(
        Pose2D(
            obstacle.x + dx * scale,
            obstacle.y + dy * scale,
            target.theta,
        )
    )
    return Pose2D(
        pushed.x,
        pushed.y,
        field.face_ball_theta(pushed.x, pushed.y, ball),
    ), True
