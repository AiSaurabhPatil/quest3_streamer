import math

import numpy as np


def _normalized_quat_wxyz(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=float).reshape(4)
    norm = np.linalg.norm(quat)
    if norm <= 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return quat / norm


def slerp_quat_wxyz(q_current, q_target, alpha):
    """Spherical interpolation for quaternions in wxyz order."""
    alpha = float(np.clip(alpha, 0.0, 1.0))
    q_current = _normalized_quat_wxyz(q_current)
    q_target = _normalized_quat_wxyz(q_target)

    dot = float(np.dot(q_current, q_target))
    if dot < 0.0:
        q_target = -q_target
        dot = -dot

    if dot > 0.9995:
        return _normalized_quat_wxyz(q_current + alpha * (q_target - q_current))

    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * alpha
    sin_theta = np.sin(theta)

    scale_current = np.cos(theta) - dot * sin_theta / sin_theta_0
    scale_target = sin_theta / sin_theta_0
    return _normalized_quat_wxyz(scale_current * q_current + scale_target * q_target)


class PositionEMA:
    """Exponential moving average for 3D positions.

    Operates in one of two modes:
      - Time-constant mode (preferred): pass tau_s. The per-step coefficient is
        computed from the actual loop dt as alpha = exp(-dt / tau), so the
        smoothing *feel* is independent of the control-loop rate. This matters
        once physics is decoupled from rendering and the loop rate changes.
      - Per-step mode (legacy): pass alpha directly (0..1). Reproduces the old
        behavior but the effective smoothing changes with loop rate.
    """

    def __init__(self, alpha: float | None = None, tau_s: float | None = None):
        self.alpha = float(np.clip(alpha, 0.0, 1.0)) if alpha is not None else None
        self.tau_s = max(1e-6, float(tau_s)) if tau_s is not None else None
        self._value: np.ndarray | None = None

    def _coefficient(self, dt_s: float | None) -> float:
        """Return the 'keep previous' weight for this step."""
        if self.tau_s is not None and dt_s is not None and dt_s > 0.0:
            return float(math.exp(-dt_s / self.tau_s))
        if self.alpha is not None:
            return self.alpha
        # No smoothing configured.
        return 0.0

    @property
    def value(self) -> np.ndarray | None:
        if self._value is None:
            return None
        return self._value.copy()

    def reset(self, value: np.ndarray | None = None) -> None:
        self._value = None if value is None else np.asarray(value, dtype=float).reshape(3).copy()

    def update(self, target: np.ndarray, dt_s: float | None = None) -> np.ndarray:
        target = np.asarray(target, dtype=float).reshape(3)
        if self._value is None:
            self._value = target.copy()
        else:
            alpha = self._coefficient(dt_s)
            self._value = alpha * self._value + (1.0 - alpha) * target
        return self._value.copy()


class OrientationSlerp:
    """Spherical-linear smoothing for orientations (quaternions, wxyz).

    Same dual-mode design as PositionEMA: time-constant mode (tau_s, preferred)
    keeps the feel stable across loop-rate changes; per-step alpha (legacy) is
    supported for backward compatibility.
    """

    def __init__(self, alpha: float | None = None, tau_s: float | None = None):
        self.alpha = float(np.clip(alpha, 0.0, 1.0)) if alpha is not None else None
        self.tau_s = max(1e-6, float(tau_s)) if tau_s is not None else None
        self._value: np.ndarray | None = None

    def _coefficient(self, dt_s: float | None) -> float:
        """Return the 'keep previous' weight for this step."""
        if self.tau_s is not None and dt_s is not None and dt_s > 0.0:
            return float(math.exp(-dt_s / self.tau_s))
        if self.alpha is not None:
            return self.alpha
        return 0.0

    @property
    def value(self) -> np.ndarray | None:
        if self._value is None:
            return None
        return self._value.copy()

    def reset(self, value: np.ndarray | None = None) -> None:
        self._value = None if value is None else _normalized_quat_wxyz(value)

    def update(self, target_wxyz: np.ndarray, dt_s: float | None = None) -> np.ndarray:
        target_wxyz = _normalized_quat_wxyz(target_wxyz)
        if self._value is None:
            self._value = target_wxyz.copy()
        else:
            alpha = self._coefficient(dt_s)
            self._value = slerp_quat_wxyz(self._value, target_wxyz, 1.0 - alpha)
        return self._value.copy()


class VelocityLimiter:
    def __init__(self, max_velocity_mps: float):
        self.max_velocity_mps = max(0.0, float(max_velocity_mps))

    def limit(
        self,
        current_xyz: np.ndarray,
        target_xyz: np.ndarray,
        dt_s: float | None,
    ) -> np.ndarray:
        current_xyz = np.asarray(current_xyz, dtype=float).reshape(3)
        target_xyz = np.asarray(target_xyz, dtype=float).reshape(3)

        if self.max_velocity_mps <= 0.0 or dt_s is None or dt_s <= 0.0:
            return target_xyz.copy()

        delta = target_xyz - current_xyz
        distance = np.linalg.norm(delta)
        max_step = self.max_velocity_mps * float(dt_s)
        if distance <= max_step or distance <= 1e-8:
            return target_xyz.copy()

        return current_xyz + delta / distance * max_step
