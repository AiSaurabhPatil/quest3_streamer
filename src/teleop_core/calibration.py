from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .controller_state import ControllerState


class HandCalibration:
    def __init__(self, samples_required: int):
        if int(samples_required) <= 0:
            raise ValueError("samples_required must be positive")
        self.samples_required = int(samples_required)
        self.reset()

    def add_sample(self, position_xyz: np.ndarray) -> bool:
        if self.complete:
            return True

        sample = np.asarray(position_xyz, dtype=float).reshape(-1)
        if sample.size != 3 or not np.all(np.isfinite(sample)):
            return False

        self._samples.append(sample.copy())
        if len(self._samples) >= self.samples_required:
            self._reference_position = np.mean(np.vstack(self._samples), axis=0)
        return self.complete

    @property
    def complete(self) -> bool:
        return self._reference_position is not None

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    @property
    def reference_position(self) -> np.ndarray | None:
        if self._reference_position is None:
            return None
        return self._reference_position.copy()

    def reset(self) -> None:
        self._samples: list[np.ndarray] = []
        self._reference_position: np.ndarray | None = None


@dataclass
class CalibrationStatus:
    left_complete: bool
    right_complete: bool
    left_samples: int
    right_samples: int


class BimanualCalibration:
    def __init__(self, samples_required: int):
        self.left = HandCalibration(samples_required)
        self.right = HandCalibration(samples_required)

    def hand_for(self, hand: str) -> HandCalibration:
        if hand == "left":
            return self.left
        if hand == "right":
            return self.right
        raise ValueError(f"Unsupported hand: {hand}")

    def update(
        self,
        left: ControllerState | None,
        right: ControllerState | None,
    ) -> None:
        self._update_hand(self.left, left)
        self._update_hand(self.right, right)

    @property
    def complete(self) -> bool:
        return self.left.complete and self.right.complete

    @property
    def status(self) -> CalibrationStatus:
        return CalibrationStatus(
            left_complete=self.left.complete,
            right_complete=self.right.complete,
            left_samples=self.left.sample_count,
            right_samples=self.right.sample_count,
        )

    def reset(self) -> None:
        self.left.reset()
        self.right.reset()

    @staticmethod
    def _update_hand(calibration: HandCalibration, state: ControllerState | None) -> None:
        if state is None or not state.has_valid_pose:
            return
        calibration.add_sample(state.pose.position_xyz)
