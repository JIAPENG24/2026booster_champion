"""Last-known-good ball buffer with constant-velocity extrapolation.

Provides a transparent grace window for the data layer: when the raw ball
observation goes stale (sensor jitter, GC latency), the buffer extrapolates
the ball position forward using constant-velocity projection from recent
history, keeping ``context.ball`` alive so SafetyGuards does not fire
StopAll prematurely.

Stages (per :meth:`update` call):
    Stage 1 — raw ball is fresh (within ``ball_fresh_sec``): record + return as-is.
    Stage 2 — raw ball stale/None but within grace (``ball_stale_grace_sec``):
        return extrapolated BallState with ``confidence=0.5``.
    Stage 3 — beyond grace: return None.

The buffer holds a fixed-size history of ``(x, y, t)`` samples for velocity
estimation via least-squares linear regression. Displacement is clamped to
``max_extrapolate_m`` to prevent chasing a position far from reality.
"""

from __future__ import annotations

from collections import deque

from .config import SoccerConfig
from .types import BallState

_MAX_HISTORY = 5
_MAX_EXTRAPOLATE_M = 0.5
_LKG_CONFIDENCE = 0.5


class BallLkgBuffer:
    """Stateful last-known-good ball tracker with constant-velocity extrapolation."""

    def __init__(self, config: SoccerConfig) -> None:
        strat = config.strategy
        self._fresh_sec: float = strat.ball_fresh_sec
        self._grace_sec: float = strat.ball_stale_grace_sec
        self._history: deque[tuple[float, float, float]] = deque(maxlen=_MAX_HISTORY)
        self._last_good: BallState | None = None

    def update(self, raw_ball: BallState | None, now: float) -> BallState | None:
        """Process a raw ball observation and return the ball to write to context.

        - Fresh raw ball: record to history, cache as last-good, return as-is.
        - Stale/None within grace: return constant-velocity extrapolation (confidence=0.5).
        - Beyond grace or never seen: return None.
        """
        if raw_ball is not None and raw_ball.is_recent(now, self._fresh_sec):
            self._last_good = raw_ball
            self._history.append((raw_ball.x, raw_ball.y, raw_ball.last_seen_at))
            return raw_ball

        if self._last_good is None:
            return None

        age = now - self._last_good.last_seen_at
        if age < 0.0 or age > self._fresh_sec + self._grace_sec:
            return None

        # Grace period: extrapolate from last-known-good.
        # Note: velocity is estimated via least-squares regression over history,
        # but displacement is projected from _last_good (not the regression-line
        # intercept). This is a first-order approximation — the discarded
        # intercept error is bounded by the regression residual (~mm at 30Hz
        # over 5 samples), negligible vs the 0.5m displacement clamp.
        vx, vy = self._estimate_velocity()
        dx = vx * age
        dy = vy * age
        disp = (dx * dx + dy * dy) ** 0.5
        if disp > _MAX_EXTRAPOLATE_M:
            scale = _MAX_EXTRAPOLATE_M / disp
            dx *= scale
            dy *= scale
        return BallState(
            x=self._last_good.x + dx,
            y=self._last_good.y + dy,
            last_seen_at=now,
            confidence=_LKG_CONFIDENCE,
        )

    def _estimate_velocity(self) -> tuple[float, float]:
        """Least-squares velocity from history. Returns (vx, vy); (0, 0) if <2 samples or degenerate timestamps."""
        if len(self._history) < 2:
            return (0.0, 0.0)
        pts = list(self._history)
        n = len(pts)
        ts = [p[2] for p in pts]
        t_mean = sum(ts) / n
        dt = [t - t_mean for t in ts]
        dt_sq = sum(d * d for d in dt)
        if dt_sq < 1e-9:
            return (0.0, 0.0)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        x_mean = sum(xs) / n
        y_mean = sum(ys) / n
        vx = sum(dt[i] * (xs[i] - x_mean) for i in range(n)) / dt_sq
        vy = sum(dt[i] * (ys[i] - y_mean) for i in range(n)) / dt_sq
        return (vx, vy)
