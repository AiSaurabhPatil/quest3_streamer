from __future__ import annotations

import os

import numpy as np
import yaml

from src.teleop_core import WorkspaceBounds

from .base import AdapterDiagnostics, CameraSpec, RobotAction, RobotAdapter


class PandaAdapter(RobotAdapter):
    def __init__(self, config: dict, project_root: str):
        self.config = config
        self.project_root = project_root
        self.articulation = None
        self.ik_solver = None
        self.dof_names: list[str] = []
        self.arm_indices: list[int] = []
        self.gripper_indices: list[int] = []
        self.last_arm_positions: np.ndarray | None = None
        self._diagnostics = AdapterDiagnostics(
            counters={
                "ik_success": 0,
                "ik_fail": 0,
            }
        )

    @classmethod
    def from_yaml(cls, config_path: str, project_root: str):
        with open(config_path, "r", encoding="utf-8") as handle:
            return cls(yaml.safe_load(handle), project_root=project_root)

    @classmethod
    def from_mapping(cls, config: dict, project_root: str):
        return cls(config, project_root=project_root)

    @property
    def usd_path(self) -> str:
        return self._resolve_path(self.config["usd"])

    @property
    def frame_name(self) -> str:
        return str(self.config["arm"]["frame_name"])

    @property
    def robot_home(self) -> np.ndarray:
        return np.asarray(self.config["teleop"]["robot_home"], dtype=float).reshape(3)

    @property
    def pos_scale(self) -> float:
        return float(self.config["teleop"].get("pos_scale", 1.0))

    @property
    def calibration_samples(self) -> int:
        return int(self.config["teleop"].get("calibration_samples", 30))

    @property
    def gripper_threshold(self) -> float:
        return float(self.config["grippers"].get("threshold", 0.3))

    @property
    def workspace_bounds(self) -> WorkspaceBounds:
        workspace = self.config["teleop"]["workspace"]
        return WorkspaceBounds(**workspace)

    def load(self, world, stage):
        from omni.isaac.franka import Franka

        self.articulation = world.scene.add(
            Franka(
                prim_path=self.config["franka_prim_path"],
                name=self.config.get("franka_name", "franka"),
            )
        )
        return self.articulation

    def initialize_ik(self):
        from omni.isaac.motion_generation import (
            LulaKinematicsSolver,
            interface_config_loader,
        )

        mg_config = interface_config_loader.load_supported_motion_policy_config(
            self.config.get("ik_robot_name", "Franka"),
            self.config.get("ik_policy_name", "RMPflow"),
        )
        self.ik_solver = LulaKinematicsSolver(
            robot_description_path=mg_config["robot_description_path"],
            urdf_path=mg_config["urdf_path"],
        )
        return True

    def initialize_joint_mappings(self) -> None:
        if self.articulation is None:
            raise RuntimeError("Robot articulation is not loaded")

        self.dof_names = list(self.articulation.dof_names)
        name_to_index = {name: index for index, name in enumerate(self.dof_names)}
        self.arm_indices = self._indices_for(self.config["arm"]["joints"], name_to_index)
        self.gripper_indices = self._indices_for(
            self.config["grippers"]["joints"],
            name_to_index,
        )

    def get_current_joint_positions(self):
        if self.articulation is None:
            return None
        return self.articulation.get_joint_positions()

    def compute_action(self, teleop_targets):
        if self.ik_solver is None:
            raise RuntimeError("IK solver is not initialized")

        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            raise RuntimeError("Robot articulation joint positions are unavailable")

        target_positions = np.asarray(current_positions, dtype=float).copy()
        ee_target = teleop_targets.ee_target
        if ee_target.valid:
            actions, success = self.ik_solver.compute_inverse_kinematics(
                target_position=ee_target.position_xyz,
                target_orientation=ee_target.orientation_wxyz,
                frame_name=self.frame_name,
            )

            if success:
                self._diagnostics.counters["ik_success"] += 1
                self.last_arm_positions = np.asarray(actions, dtype=float).reshape(-1)[
                    : len(self.arm_indices)
                ]
            else:
                self._diagnostics.counters["ik_fail"] += 1

        if self.last_arm_positions is not None:
            for offset, joint_index in enumerate(self.arm_indices):
                if offset < self.last_arm_positions.size:
                    target_positions[joint_index] = self.last_arm_positions[offset]

        gripper_target = teleop_targets.gripper_target
        gripper_position = (
            float(self.config["grippers"]["closed_position"])
            if gripper_target.closed
            else float(self.config["grippers"]["open_position"])
        )
        for joint_index in self.gripper_indices:
            target_positions[joint_index] = gripper_position

        return RobotAction(joint_positions=target_positions)

    def apply_action(self, action):
        from omni.isaac.core.utils.types import ArticulationAction

        if self.articulation is None:
            raise RuntimeError("Robot articulation is not loaded")
        self.articulation.apply_action(
            ArticulationAction(joint_positions=action.joint_positions)
        )

    def get_joint_names(self) -> list[str]:
        return list(self.dof_names)

    def get_camera_specs(self) -> dict[str, CameraSpec]:
        return {
            name: CameraSpec(
                name=name,
                prim_path=spec["prim_path"],
                topic=spec["topic"],
            )
            for name, spec in self.config.get("cameras", {}).items()
        }

    def get_viewport_cameras(self) -> list[tuple[str, str]]:
        cameras = [("Perspective", "/OmniverseKit_Persp")]
        for name, spec in self.config.get("cameras", {}).items():
            cameras.append((name.replace("_", " ").title(), spec["prim_path"]))
        return cameras

    def get_diagnostics(self) -> AdapterDiagnostics:
        return AdapterDiagnostics(
            counters=dict(self._diagnostics.counters),
            details={"frame_name": self.frame_name},
        )

    def _resolve_path(self, path: str) -> str:
        if os.path.isabs(path):
            return path
        return os.path.join(self.project_root, path)

    @staticmethod
    def _indices_for(joint_names: list[str], name_to_index: dict[str, int]) -> list[int]:
        return [name_to_index[name] for name in joint_names if name in name_to_index]
