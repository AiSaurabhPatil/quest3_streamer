from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from .controller_state import ControllerState
from .filters import VelocityLimiter


@dataclass
class WorkspaceBounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def clamp(self, position_xyz: np.ndarray) -> np.ndarray:
        position_xyz = np.asarray(position_xyz, dtype=float).reshape(3)
        return np.array(
            [
                np.clip(position_xyz[0], self.x_min, self.x_max),
                np.clip(position_xyz[1], self.y_min, self.y_max),
                np.clip(position_xyz[2], self.z_min, self.z_max),
            ],
            dtype=float,
        )


@dataclass
class TargetSafetyConfig:
    workspace_bounds: WorkspaceBounds | None = None
    max_translation_step_m: float | None = None
    max_velocity_mps: float | None = None
    stale_timeout_s: float | None = None


class TargetSafety:
    def __init__(self, config: TargetSafetyConfig):
        self.config = config
        self._velocity_limiter = (
            VelocityLimiter(config.max_velocity_mps)
            if config.max_velocity_mps is not None
            else None
        )
        self._last_valid_position: np.ndarray | None = None

    def is_state_stale(
        self,
        controller_state: ControllerState | None,
        now_s: float | None = None,
    ) -> bool:
        if controller_state is None or not controller_state.has_valid_pose:
            return True
        if self.config.stale_timeout_s is None:
            return False
        if now_s is None:
            now_s = time.monotonic()
        return controller_state.age_s(now_s) > self.config.stale_timeout_s

    def apply_position(
        self,
        target_xyz: np.ndarray,
        previous_xyz: np.ndarray | None = None,
        dt_s: float | None = None,
    ) -> tuple[np.ndarray, bool]:
        fallback = self._fallback_position(previous_xyz)
        target_xyz = np.asarray(target_xyz, dtype=float).reshape(3)

        if not np.all(np.isfinite(target_xyz)):
            return fallback, False

        safe_target = target_xyz.copy()

        if self.config.workspace_bounds is not None:
            safe_target = self.config.workspace_bounds.clamp(safe_target)

        if previous_xyz is not None:
            previous_xyz = np.asarray(previous_xyz, dtype=float).reshape(3)
            if self.config.max_translation_step_m is not None:
                jump = np.linalg.norm(safe_target - previous_xyz)
                if jump > self.config.max_translation_step_m:
                    return self._remember(previous_xyz), False
            if self._velocity_limiter is not None:
                safe_target = self._velocity_limiter.limit(previous_xyz, safe_target, dt_s)

        return self._remember(safe_target), True

    def _fallback_position(self, previous_xyz: np.ndarray | None) -> np.ndarray:
        if previous_xyz is not None:
            return np.asarray(previous_xyz, dtype=float).reshape(3).copy()
        if self._last_valid_position is not None:
            return self._last_valid_position.copy()
        return np.zeros(3, dtype=float)

    def _remember(self, position_xyz: np.ndarray) -> np.ndarray:
        self._last_valid_position = np.asarray(position_xyz, dtype=float).reshape(3).copy()
        return self._last_valid_position.copy()
