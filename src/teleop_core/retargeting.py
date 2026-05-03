from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


def _as_vector(values, size: int, fallback) -> np.ndarray:
    array = np.asarray(values, dtype=float).reshape(-1)
    if array.size != size:
        array = np.asarray(fallback, dtype=float)
    return array.copy()


@dataclass
class EndEffectorTarget:
    position_xyz: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    orientation_wxyz: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    )
    valid: bool = True

    def __post_init__(self):
        self.position_xyz = _as_vector(self.position_xyz, 3, np.zeros(3, dtype=float))
        self.orientation_wxyz = _as_vector(
            self.orientation_wxyz,
            4,
            np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        )
        self.valid = bool(self.valid)


@dataclass
class GripperTarget:
    closed: bool = False
    analog_value: float = 0.0

    def __post_init__(self):
        self.closed = bool(self.closed)
        self.analog_value = float(self.analog_value)


@dataclass
class SingleArmTeleopTargets:
    ee_target: EndEffectorTarget = field(default_factory=EndEffectorTarget)
    gripper_target: GripperTarget = field(default_factory=GripperTarget)


@dataclass
class BimanualTeleopTargets:
    left_ee: EndEffectorTarget = field(default_factory=EndEffectorTarget)
    right_ee: EndEffectorTarget = field(default_factory=EndEffectorTarget)
    left_gripper: GripperTarget = field(default_factory=GripperTarget)
    right_gripper: GripperTarget = field(default_factory=GripperTarget)
