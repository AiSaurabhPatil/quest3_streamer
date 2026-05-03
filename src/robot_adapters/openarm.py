from __future__ import annotations

from dataclasses import dataclass
import os

import numpy as np
import yaml

from .base import AdapterDiagnostics, CameraSpec, RobotAction, RobotAdapter


@dataclass
class _ArmRuntime:
    frame_name: str
    joint_names: list[str]
    preferred_config: np.ndarray
    last_arm_positions: np.ndarray | None = None


class OpenArmAdapter(RobotAdapter):
    def __init__(self, config: dict, project_root: str):
        self.config = config
        self.project_root = project_root
        self.articulation = None
        self.robot_prim_path: str | None = None
        self.left_ik_solver = None
        self.right_ik_solver = None
        self.ik_enabled = False
        self.dof_names: list[str] = []
        self.left_arm_indices: list[int] = []
        self.right_arm_indices: list[int] = []
        self.left_gripper_indices: list[int] = []
        self.right_gripper_indices: list[int] = []
        self.left_runtime = _ArmRuntime(
            frame_name=self.config["left_arm"]["frame_name"],
            joint_names=list(self.config["left_arm"]["joints"]),
            preferred_config=np.asarray(
                self.config["left_arm"]["preferred_config"],
                dtype=float,
            ).reshape(-1),
        )
        self.right_runtime = _ArmRuntime(
            frame_name=self.config["right_arm"]["frame_name"],
            joint_names=list(self.config["right_arm"]["joints"]),
            preferred_config=np.asarray(
                self.config["right_arm"]["preferred_config"],
                dtype=float,
            ).reshape(-1),
        )
        self._smoothed_left_gripper = float(self.config["grippers"]["open_position"])
        self._smoothed_right_gripper = float(self.config["grippers"]["open_position"])
        self._diagnostics = AdapterDiagnostics(
            counters={
                "left_ik_success": 0,
                "left_ik_fail": 0,
                "right_ik_success": 0,
                "right_ik_fail": 0,
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
    def left_workspace_offset(self) -> np.ndarray:
        return np.asarray(
            self.config["left_arm"].get("workspace_offset", [0.0, 0.15, 0.0]),
            dtype=float,
        ).reshape(3)

    @property
    def right_workspace_offset(self) -> np.ndarray:
        return np.asarray(
            self.config["right_arm"].get("workspace_offset", [0.0, -0.15, 0.0]),
            dtype=float,
        ).reshape(3)

    @property
    def gripper_threshold(self) -> float:
        return float(self.config["grippers"].get("threshold", 0.5))

    @property
    def gripper_speed(self) -> float:
        return float(self.config["grippers"]["speed"])

    def load(self, world, stage):
        from omni.isaac.core.articulations import Articulation

        robot_prim_path = None
        for path in self.config.get("prim_search_paths", []):
            robot_prim = stage.GetPrimAtPath(path)
            if robot_prim.IsValid():
                robot_prim_path = path
                break

        if robot_prim_path is None:
            available = [str(prim.GetPath()) for prim in stage.GetPseudoRoot().GetChildren()]
            raise RuntimeError(
                "Could not find OpenArm robot in the USD stage. "
                f"Checked: {self.config.get('prim_search_paths', [])}. "
                f"Available roots: {available}"
            )

        self.robot_prim_path = robot_prim_path
        self.articulation = world.scene.add(
            Articulation(
                prim_path=robot_prim_path,
                name="openarm",
            )
        )
        return self.articulation

    def initialize_ik(self):
        from omni.isaac.motion_generation import LulaKinematicsSolver

        left_robot_desc_path = os.path.join(
            self._resolve_path(self.config["left_arm_config"]),
            "robot_descriptor.yaml",
        )
        right_robot_desc_path = os.path.join(
            self._resolve_path(self.config["right_arm_config"]),
            "robot_descriptor.yaml",
        )
        urdf_path = self._resolve_path(self.config["urdf"])

        try:
            self.left_ik_solver = LulaKinematicsSolver(
                robot_description_path=left_robot_desc_path,
                urdf_path=urdf_path,
            )
            self.right_ik_solver = LulaKinematicsSolver(
                robot_description_path=right_robot_desc_path,
                urdf_path=urdf_path,
            )
            self.ik_enabled = True
        except Exception as exc:
            self.left_ik_solver = None
            self.right_ik_solver = None
            self.ik_enabled = False
            self._diagnostics.details["ik_init_error"] = str(exc)
        return self.ik_enabled

    def initialize_joint_mappings(self) -> None:
        if self.articulation is None:
            raise RuntimeError("Robot articulation is not loaded")

        self.dof_names = list(self.articulation.dof_names)
        name_to_index = {name: index for index, name in enumerate(self.dof_names)}

        self.left_arm_indices = self._indices_for(self.left_runtime.joint_names, name_to_index)
        self.right_arm_indices = self._indices_for(self.right_runtime.joint_names, name_to_index)
        self.left_gripper_indices = self._indices_for(
            self.config["grippers"]["left_joints"],
            name_to_index,
        )
        self.right_gripper_indices = self._indices_for(
            self.config["grippers"]["right_joints"],
            name_to_index,
        )

    def get_current_joint_positions(self):
        if self.articulation is None:
            return None
        return self.articulation.get_joint_positions()

    def compute_action(self, teleop_targets):
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            raise RuntimeError("Robot articulation joint positions are unavailable")

        target_positions = np.asarray(current_positions, dtype=float).copy()

        if self.ik_enabled:
            self._apply_arm_ik(
                solver=self.left_ik_solver,
                runtime=self.left_runtime,
                indices=self.left_arm_indices,
                ee_target=teleop_targets.left_ee,
                target_positions=target_positions,
                success_key="left_ik_success",
                fail_key="left_ik_fail",
            )
            self._apply_arm_ik(
                solver=self.right_ik_solver,
                runtime=self.right_runtime,
                indices=self.right_arm_indices,
                ee_target=teleop_targets.right_ee,
                target_positions=target_positions,
                success_key="right_ik_success",
                fail_key="right_ik_fail",
            )

        self._smoothed_left_gripper = self._step_gripper(
            self._smoothed_left_gripper,
            teleop_targets.left_gripper.closed,
        )
        self._smoothed_right_gripper = self._step_gripper(
            self._smoothed_right_gripper,
            teleop_targets.right_gripper.closed,
        )

        for index in self.left_gripper_indices:
            target_positions[index] = self._smoothed_left_gripper
        for index in self.right_gripper_indices:
            target_positions[index] = self._smoothed_right_gripper

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
        diagnostics = AdapterDiagnostics(
            counters=dict(self._diagnostics.counters),
            details=dict(self._diagnostics.details),
        )
        diagnostics.details["robot_prim_path"] = self.robot_prim_path
        diagnostics.details["ik_enabled"] = self.ik_enabled
        return diagnostics

    def _apply_arm_ik(
        self,
        solver,
        runtime: _ArmRuntime,
        indices: list[int],
        ee_target,
        target_positions: np.ndarray,
        success_key: str,
        fail_key: str,
    ) -> None:
        if solver is None or not ee_target.valid:
            return

        warm_start = (
            runtime.last_arm_positions
            if runtime.last_arm_positions is not None
            else runtime.preferred_config
        )
        actions, success = solver.compute_inverse_kinematics(
            target_position=ee_target.position_xyz,
            target_orientation=ee_target.orientation_wxyz,
            frame_name=runtime.frame_name,
            warm_start=warm_start,
        )

        if success:
            self._diagnostics.counters[success_key] += 1
            arm_positions = np.asarray(actions, dtype=float).reshape(-1)[: len(indices)]
            runtime.last_arm_positions = arm_positions.copy()
            for offset, joint_index in enumerate(indices):
                if offset < arm_positions.size:
                    target_positions[joint_index] = arm_positions[offset]
            return

        self._diagnostics.counters[fail_key] += 1
        if runtime.last_arm_positions is None:
            return
        for offset, joint_index in enumerate(indices):
            if offset < runtime.last_arm_positions.size:
                target_positions[joint_index] = runtime.last_arm_positions[offset]

    def _step_gripper(self, current: float, closed: bool) -> float:
        open_position = float(self.config["grippers"]["open_position"])
        closed_position = float(self.config["grippers"]["closed_position"])
        target = closed_position if closed else open_position

        if current < target:
            return min(current + self.gripper_speed, target)
        if current > target:
            return max(current - self.gripper_speed, target)
        return current

    def _resolve_path(self, path: str) -> str:
        if os.path.isabs(path):
            return path
        return os.path.join(self.project_root, path)

    @staticmethod
    def _indices_for(joint_names: list[str], name_to_index: dict[str, int]) -> list[int]:
        return [name_to_index[name] for name in joint_names if name in name_to_index]
