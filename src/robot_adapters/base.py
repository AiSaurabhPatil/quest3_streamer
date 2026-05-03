from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class CameraSpec:
    name: str
    prim_path: str
    topic: str


@dataclass
class RobotAction:
    joint_positions: np.ndarray

    def __post_init__(self):
        self.joint_positions = np.asarray(self.joint_positions, dtype=float).reshape(-1)


@dataclass
class AdapterDiagnostics:
    counters: dict[str, int] = field(default_factory=dict)
    details: dict[str, Any] = field(default_factory=dict)


class RobotAdapter(ABC):
    @abstractmethod
    def load(self, world, stage):
        """Load or attach to robot articulation."""

    @abstractmethod
    def initialize_ik(self):
        """Initialize IK solvers."""

    @abstractmethod
    def get_current_joint_positions(self):
        """Return current articulation joint positions."""

    @abstractmethod
    def compute_action(self, teleop_targets):
        """Return a robot action from normalized teleop targets."""

    @abstractmethod
    def apply_action(self, action):
        """Apply a previously computed action to the robot articulation."""

    @abstractmethod
    def get_joint_names(self) -> list[str]:
        """Return articulation joint names in simulator order."""

    def get_camera_specs(self) -> dict[str, CameraSpec]:
        return {}

    def get_viewport_cameras(self) -> list[tuple[str, str]]:
        return []

    def get_diagnostics(self) -> AdapterDiagnostics:
        return AdapterDiagnostics()
