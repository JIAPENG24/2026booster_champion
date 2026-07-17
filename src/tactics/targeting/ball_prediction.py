"""Ball trajectory prediction with PID-smoothed velocity estimation and friction-based extrapolation.

Velocity is estimated from a 3-frame difference with friction compensation
(consistent with the exponential friction model, unlike the old least-squares
regression which assumed constant velocity), then smoothed with a PID filter
that drives the estimate toward the measured value.  Trajectory extrapolation
uses an exponential friction model::

    v(t) = v0 * exp(-mu * t)
    x(t) = x0 + (v0 / mu) * (1 - exp(-mu * t))

The friction coefficient ``mu`` is estimated online from deceleration patterns in the
ball's velocity history, with an adaptive update rate (fast initially, stable after).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional


@dataclass
class BallTrajectory:
    """Predicted ball state at a specific future time."""

    x: float
    y: float
    vx: float
    vy: float


class BallPredictor:
    """N-frame ball history with PID-smoothed velocity and online friction estimation.

    Parameters
    ----------
    history_size:
        Number of recent ball samples to retain for linear-regression velocity
        estimation.
    kp, ki, kd:
        PID gains for the velocity smoothing filter.  The filter drives the
        smoothed velocity toward the regression-measured velocity.
    friction_init:
        Initial friction coefficient (1/s).  Updated online from deceleration.
    max_horizon_sec:
        Maximum prediction horizon (s).  ``predict_at`` clamps to this value.
    """

    _INTEGRAL_CLAMP = 5.0  # anti-windup limit for PID integral term
    _MU_CLAMP_MIN = 0.01  # friction coefficient lower bound
    _MU_CLAMP_MAX = 5.0  # friction coefficient upper bound
    _SPEED_THRESHOLD = 0.3  # below this speed (m/s), friction estimation is skipped
    _TIME_EPS = 1e-6  # minimum time delta for valid velocity computation
    _MAX_REST_DISTANCE = 16.0  # clamp rest-point prediction to this radius (m); field_length(14)+2

    def __init__(
        self,
        history_size: int = 20,
        kp: float = 0.6,
        ki: float = 0.05,
        kd: float = 0.1,
        friction_init: float = 0.3,
        max_horizon_sec: float = 2.0,
    ):
        self._history: deque[tuple[float, float, float]] = deque(maxlen=history_size)
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._mu = friction_init
        self._max_horizon = max_horizon_sec

        # Smoothed state
        self._smooth_vx: float = 0.0
        self._smooth_vy: float = 0.0
        self._smooth_ax: float = 0.0
        self._smooth_ay: float = 0.0

        # PID state
        self._integral_vx: float = 0.0
        self._integral_vy: float = 0.0
        self._prev_error_vx: float = 0.0
        self._prev_error_vy: float = 0.0
        self._prev_time: float = 0.0

        # Friction adaptation counter (adaptive rate: fast initially, stable later)
        self._update_count: int = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Clear all history and reset PID state (call on game-state changes)."""
        self._history.clear()
        self._smooth_vx = 0.0
        self._smooth_vy = 0.0
        self._smooth_ax = 0.0
        self._smooth_ay = 0.0
        self._integral_vx = 0.0
        self._integral_vy = 0.0
        self._prev_error_vx = 0.0
        self._prev_error_vy = 0.0
        self._prev_time = 0.0
        self._update_count = 0

    def update(self, x: float, y: float, timestamp: float) -> None:
        """Add a new ball sample and recompute smoothed velocity/acceleration."""
        self._history.append((x, y, timestamp))
        if len(self._history) < 2:
            self._prev_time = timestamp
            return

        # Step 1 — raw velocity via linear regression
        raw_vx, raw_vy = self._regress_velocity()

        # Step 2 — dt and acceleration
        dt = timestamp - self._prev_time
        if dt > 0.5:
            # Stale frame: decay everything and reset PID accumulator
            self._smooth_vx *= 0.9
            self._smooth_vy *= 0.9
            self._smooth_ax *= 0.9
            self._smooth_ay *= 0.9
            self._integral_vx *= 0.5
            self._integral_vy *= 0.5
            self._prev_time = timestamp
            return

        dt = max(dt, self._TIME_EPS)
        raw_ax = (raw_vx - self._smooth_vx) / dt
        raw_ay = (raw_vy - self._smooth_vy) / dt

        # Step 3 — PID velocity smoothing
        error_vx = raw_vx - self._smooth_vx
        error_vy = raw_vy - self._smooth_vy

        self._integral_vx = self._clamp(
            self._integral_vx + error_vx * dt,
            -self._INTEGRAL_CLAMP,
            self._INTEGRAL_CLAMP,
        )
        self._integral_vy = self._clamp(
            self._integral_vy + error_vy * dt,
            -self._INTEGRAL_CLAMP,
            self._INTEGRAL_CLAMP,
        )

        # Discrete D term: delta of error (not divided by dt — avoids spikes
        # when the measurement jumps sharply between frames).
        deriv_vx = error_vx - self._prev_error_vx
        deriv_vy = error_vy - self._prev_error_vy

        self._smooth_vx += (
            self._kp * error_vx + self._ki * self._integral_vx + self._kd * deriv_vx
        )
        self._smooth_vy += (
            self._kp * error_vy + self._ki * self._integral_vy + self._kd * deriv_vy
        )

        # Smooth acceleration (simple IIR)
        self._smooth_ax = 0.5 * self._smooth_ax + 0.5 * raw_ax
        self._smooth_ay = 0.5 * self._smooth_ay + 0.5 * raw_ay

        self._prev_error_vx = error_vx
        self._prev_error_vy = error_vy
        self._prev_time = timestamp

        # Step 4 — online friction estimation
        self._update_friction()

    def predict_at(self, t_sec: float) -> BallTrajectory:
        """Predict ball position and velocity at *t_sec* seconds in the future.

        Uses the exponential friction model ``v(t) = v0 * exp(-mu * t)``.
        """
        t = max(0.0, min(t_sec, self._max_horizon))
        mu = max(self._mu, self._MU_CLAMP_MIN)

        if len(self._history) == 0:
            return BallTrajectory(0.0, 0.0, 0.0, 0.0)

        latest_x, latest_y, _ = self._history[-1]

        decay = math.exp(-mu * t)
        pred_vx = self._smooth_vx * decay
        pred_vy = self._smooth_vy * decay

        pos_factor = (1.0 - decay) / mu
        pred_x = latest_x + self._smooth_vx * pos_factor
        pred_y = latest_y + self._smooth_vy * pos_factor

        return BallTrajectory(pred_x, pred_y, pred_vx, pred_vy)

    def predict_rest_point(self) -> tuple[float, float]:
        """Predict where the ball stops (velocity -> 0) under friction.

        With exponential decay, rest point = pos + v / mu (as t -> inf).
        Clamped to ``_MAX_REST_DISTANCE`` to avoid extreme values when the
        online friction estimate is very low.
        """
        mu = max(self._mu, self._MU_CLAMP_MIN)
        if len(self._history) == 0:
            return (0.0, 0.0)
        latest_x, latest_y, _ = self._history[-1]
        rest_x = latest_x + self._smooth_vx / mu
        rest_y = latest_y + self._smooth_vy / mu

        dx = rest_x - latest_x
        dy = rest_y - latest_y
        dist = math.hypot(dx, dy)
        if dist > self._MAX_REST_DISTANCE:
            scale = self._MAX_REST_DISTANCE / dist
            rest_x = latest_x + dx * scale
            rest_y = latest_y + dy * scale

        return (rest_x, rest_y)

    def predict_goal_crossing(
        self,
        goal_x: float,
        goal_half_width: float,
    ) -> Optional[tuple[float, float, bool]]:
        """Predict where the ball crosses the goal line ``x = goal_x``.

        Returns ``(t_sec, y_at_crossing, is_on_target)`` if the ball will reach the
        goal line, or ``None`` if friction stops it first or the ball is moving away.
        """
        if len(self._history) == 0:
            return None

        latest_x, latest_y, _ = self._history[-1]
        mu = max(self._mu, self._MU_CLAMP_MIN)

        if abs(self._smooth_vx) < self._TIME_EPS:
            return None

        delta_x = goal_x - latest_x

        # Ball must be moving toward the goal line
        if delta_x * self._smooth_vx <= 0:
            return None

        # Solve: latest_x + (vx/mu)(1 - exp(-mu*t)) = goal_x
        ratio = delta_x * mu / self._smooth_vx
        if ratio >= 1.0:
            # Friction stops the ball before it reaches the goal line
            return None

        t = -math.log(1.0 - ratio) / mu
        if t > self._max_horizon * 3:
            return None

        decay = math.exp(-mu * t)
        pos_factor = (1.0 - decay) / mu
        y_cross = latest_y + self._smooth_vy * pos_factor
        is_on_target = abs(y_cross) <= goal_half_width

        return (t, y_cross, is_on_target)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def smooth_vx(self) -> float:
        return self._smooth_vx

    @property
    def smooth_vy(self) -> float:
        return self._smooth_vy

    @property
    def friction(self) -> float:
        return self._mu

    @property
    def has_history(self) -> bool:
        return len(self._history) >= 2

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _regress_velocity(self) -> tuple[float, float]:
        """Estimate instantaneous velocity using a 3-frame difference with
        friction compensation.

        Replaces the old least-squares regression (which assumed constant
        velocity, contradicting the friction model).  Uses the last 3
        positions to compute an average velocity, then compensates for
        exponential friction decay over the interval::

            v(t1) = v_avg * mu*dt / (exp(mu*dt) - 1)

        This recovers the instantaneous velocity at the latest timestamp
        under the model ``v(t) = v0 * exp(-mu*t)``.
        """
        n = len(self._history)
        if n < 3:
            if n < 2:
                return 0.0, 0.0
            # Fall back to 2-point difference for the first sample
            x0, y0, t0 = self._history[-2]
            x1, y1, t1 = self._history[-1]
            dt = t1 - t0
            if dt < self._TIME_EPS:
                return 0.0, 0.0
            return (x1 - x0) / dt, (y1 - y0) / dt

        x0, y0, t0 = self._history[-3]
        x1, y1, t1 = self._history[-1]
        dt = t1 - t0
        if dt < self._TIME_EPS:
            return 0.0, 0.0

        # Average velocity over the 3-frame window
        vx_avg = (x1 - x0) / dt
        vy_avg = (y1 - y0) / dt

        # Friction compensation:
        # Under v(t)=v0*exp(-mu*t), the average velocity over [t0, t1] is:
        #   v_avg = v0 * (1-exp(-mu*dt)) / (mu*dt)
        # Instantaneous velocity at t1 is v(t1) = v0 * exp(-mu*dt), so:
        #   v(t1) = v_avg * mu*dt * exp(-mu*dt) / (1-exp(-mu*dt))
        #          = v_avg * mu*dt / (exp(mu*dt) - 1)
        mu = max(self._mu, self._MU_CLAMP_MIN)
        exp_mu_dt = math.exp(mu * dt)
        if exp_mu_dt > 1.001:
            comp = mu * dt / (exp_mu_dt - 1.0)
            vx_avg *= comp
            vy_avg *= comp

        return vx_avg, vy_avg

    def _update_friction(self) -> None:
        """Estimate friction coefficient from velocity decay.

        For rolling friction ``dv/dt = -mu * v``, the deceleration along the
        velocity direction is ``mu * speed``.  We recover ``mu`` from the
        smoothed acceleration projected onto the velocity vector.
        """
        speed = math.hypot(self._smooth_vx, self._smooth_vy)
        if speed < self._SPEED_THRESHOLD:
            return

        # Project acceleration onto velocity direction (negative = decelerating)
        decel = -(
            self._smooth_vx * self._smooth_ax + self._smooth_vy * self._smooth_ay
        ) / (speed * speed)

        if decel <= 0:
            # Ball is accelerating or turning — skip update
            return

        if self._MU_CLAMP_MIN <= decel <= self._MU_CLAMP_MAX:
            self._update_count += 1
            # Fast convergence during the first 30 frames, then stable
            rate = 0.3 if self._update_count < 30 else 0.1
            self._mu = (1.0 - rate) * self._mu + rate * decel

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))
