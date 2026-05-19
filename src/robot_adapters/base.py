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

    def get_last_ik_debug(self) -> dict[str, dict[str, object]]:
        return {}

    def get_debug_end_effector_positions(self, joint_positions=None) -> dict[str, np.ndarray]:
        return {}

    def get_recording_robot_type(self) -> str:
        return self.__class__.__name__.replace("Adapter", "").lower()

    def get_articulation_vector_by_joint_names(
        self,
        joint_names: list[str] | tuple[str, ...],
        joint_positions,
    ) -> np.ndarray:
        positions = np.asarray(joint_positions, dtype=np.float32).reshape(-1)
        name_to_index = {name: idx for idx, name in enumerate(self.get_joint_names())}
        missing = [name for name in joint_names if name not in name_to_index]
        if missing:
            raise KeyError(f"Unknown articulation joints for recording: {missing}")
        return np.asarray([positions[name_to_index[name]] for name in joint_names], dtype=np.float32)

    def get_recording_vector(
        self,
        *,
        vector_config,
        current_joint_positions,
        commanded_action,
        teleop_targets=None,
    ) -> tuple[list[str], np.ndarray]:
        mode = getattr(vector_config, "mode", None) or vector_config.get("mode", "articulation_joints")
        if mode == "articulation_joints":
            joint_names = list(self.get_joint_names())
            source_positions = current_joint_positions
            if current_joint_positions is None:
                source_positions = commanded_action.joint_positions
            return joint_names, self.get_articulation_vector_by_joint_names(joint_names, source_positions)

        raise NotImplementedError(
            f"{self.__class__.__name__} does not support recording mode '{mode}'"
        )
