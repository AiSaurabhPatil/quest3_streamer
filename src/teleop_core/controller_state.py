from dataclasses import dataclass, field
from typing import Optional

import numpy as np


def _as_vector(values, size: int, fallback) -> np.ndarray:
    array = np.asarray(values, dtype=float).reshape(-1)
    if array.size != size:
        array = np.asarray(fallback, dtype=float)
    return array.copy()


@dataclass
class ControllerButtons:
    primary: bool = False
    secondary: bool = False
    menu: bool = False
    stick_click: bool = False


@dataclass
class ControllerAxes:
    trigger: float = 0.0
    squeeze: float = 0.0
    thumbstick_x: float = 0.0
    thumbstick_y: float = 0.0


@dataclass
class ControllerPose:
    position_xyz: np.ndarray
    orientation_xyzw: np.ndarray
    valid: bool = True

    def __post_init__(self):
        self.position_xyz = _as_vector(self.position_xyz, 3, np.zeros(3, dtype=float))
        self.orientation_xyzw = _as_vector(
            self.orientation_xyzw,
            4,
            np.array([0.0, 0.0, 0.0, 1.0], dtype=float),
        )
        self.valid = bool(self.valid)


@dataclass
class ControllerState:
    hand: str
    sequence: int = -1
    source_timestamp: float = 0.0
    receive_time_s: float = 0.0
    pose: Optional[ControllerPose] = None
    axes: ControllerAxes = field(default_factory=ControllerAxes)
    buttons: ControllerButtons = field(default_factory=ControllerButtons)
    client_epoch_ms: float | None = None
    ingress_receive_epoch_ms: float | None = None
    remote_receive_epoch_ms: float | None = None
    ros_publish_epoch_ms: float | None = None
    control_receive_epoch_ms: float | None = None
    isaac_apply_epoch_ms: float | None = None

    @property
    def has_valid_pose(self) -> bool:
        return self.pose is not None and self.pose.valid

    def age_s(self, now_s: float) -> float:
        return max(0.0, float(now_s) - float(self.receive_time_s))
