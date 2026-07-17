"""Last-known-good robot pose buffer with constant-velocity extrapolation.

Provides a transparent grace window for the data layer: when a robot's raw
pose observation goes stale (sensor jitter, GC latency), the buffer
extrapolates the pose forward using constant-velocity projection (including
heading) from recent history, keeping ``robot.pose`` alive so the motion
controller does not stop that player prematurely.

Stages (per :meth:`update` call, applied to each robot in the dict):
    Stage 1 — raw pose is fresh (within ``robot_pose_fresh_sec``): record + keep as-is.
    Stage 2 — raw pose stale/None but within grace (``robot_pose_stale_grace_sec``):
        replace with extrapolated Pose2D projected from last-known-good.
    Stage 3 — beyond grace or never seen: set pose to None.

The buffer holds a fixed-size history of ``(x, y, theta, t)`` samples per
robot for velocity estimation via least-squares linear regression. Theta is
unwrapped before regression to handle the ±π boundary. Linear displacement
(x, y) is clamped to ``max_extrapolate_m`` to prevent drifting far from
reality; theta is unclamped since angular overshoot self-corrects on the
next fresh sample.

Tighter defaults than :class:`BallLkgBuffer` because robot motion under
active control is less predictable than a friction-decayed ball.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from .config import SoccerConfig
from .types import Pose2D, RobotState

_MAX_HISTORY = 5
_MAX_EXTRAPOLATE_M = 0.3


def _normalize_angle(angle: float) -> float:
    """Wrap angle to [-π, π].

    Inlined (rather than imported from ``tactics.geometry``) to preserve the
    soccer_framework → tactics dependency direction.
    """

    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class _RobotTracker:
    """Per-robot LKG state."""

    history: deque
    last_good: Pose2D | None = None
    last_good_t: float = 0.0


class PoseLkgBuffer:
    """Stateful last-known-good pose tracker with constant-velocity extrapolation.

    One instance manages a whole dict of robots (keyed by ``player_id``). Use
    separate instances for teammates vs opponents, since their ``player_id``
    namespaces may overlap.
    """

    def __init__(self, config: SoccerConfig) -> None:
        strat = config.strategy
        self._fresh_sec: float = strat.robot_pose_fresh_sec
        self._grace_sec: float = strat.robot_pose_stale_grace_sec
        self._trackers: dict[int, _RobotTracker] = {}

    def update(self, robots: dict[int, RobotState], now: float) -> None:
        """Apply freshness + grace filtering to each robot, mutating ``robot.pose`` in place.

        - Fresh pose: record to history, cache as last-good, keep as-is.
        - Stale/None within grace: replace with constant-velocity extrapolation.
        - Beyond grace or never seen: set pose to None.
        """

        for player_id, robot in robots.items():
            tracker = self._trackers.get(player_id)
            if tracker is None:
                tracker = _RobotTracker(history=deque(maxlen=_MAX_HISTORY))
                self._trackers[player_id] = tracker

            pose = robot.pose
            if pose is not None and robot.is_recent(now, self._fresh_sec):
                tracker.last_good = pose
                tracker.last_good_t = robot.last_seen_at
                tracker.history.append((pose.x, pose.y, pose.theta, robot.last_seen_at))
                continue

            # Stale or None — try grace extrapolation.
            if tracker.last_good is None:
                robot.pose = None
                continue

            age = now - tracker.last_good_t
            if age < 0.0 or age > self._fresh_sec + self._grace_sec:
                robot.pose = None
                continue

            # Grace period: extrapolate from last-known-good.
            vx, vy, vtheta = self._estimate_velocity(tracker)
            dx = vx * age
            dy = vy * age
            disp = math.hypot(dx, dy)
            if disp > _MAX_EXTRAPOLATE_M:
                scale = _MAX_EXTRAPOLATE_M / disp
                dx *= scale
                dy *= scale
            robot.pose = Pose2D(
                x=tracker.last_good.x + dx,
                y=tracker.last_good.y + dy,
                theta=_normalize_angle(tracker.last_good.theta + vtheta * age),
            )

    @staticmethod
    def _estimate_velocity(tracker: _RobotTracker) -> tuple[float, float, float]:
        """Least-squares velocity from history.

        Returns ``(vx, vy, vtheta)``; ``(0, 0, 0)`` if fewer than 2 samples or
        degenerate timestamps. Theta is unwrapped before regression to handle
        the ±π boundary.
        """

        if len(tracker.history) < 2:
            return (0.0, 0.0, 0.0)
        pts = list(tracker.history)
        n = len(pts)
        ts = [p[3] for p in pts]
        t_mean = sum(ts) / n
        dt = [t - t_mean for t in ts]
        dt_sq = sum(d * d for d in dt)
        if dt_sq < 1e-9:
            return (0.0, 0.0, 0.0)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        # Unwrap theta: cumulative sum of wrapped consecutive deltas.
        raw_thetas = [p[2] for p in pts]
        unwrapped = [raw_thetas[0]]
        for i in range(1, n):
            unwrapped.append(
                unwrapped[i - 1] + _normalize_angle(raw_thetas[i] - raw_thetas[i - 1])
            )
        x_mean = sum(xs) / n
        y_mean = sum(ys) / n
        th_mean = sum(unwrapped) / n
        vx = sum(dt[i] * (xs[i] - x_mean) for i in range(n)) / dt_sq
        vy = sum(dt[i] * (ys[i] - y_mean) for i in range(n)) / dt_sq
        vtheta = sum(dt[i] * (unwrapped[i] - th_mean) for i in range(n)) / dt_sq
        return (vx, vy, vtheta)
