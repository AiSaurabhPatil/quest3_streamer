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
                "left_ik_step_limited": 0,
                "right_ik_step_limited": 0,
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

    @property
    def robot_label(self) -> str:
        return str(
            self.config.get("display_name")
            or self.config.get("robot_type")
            or self.__class__.__name__.replace("Adapter", "")
        )

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
                f"Could not find {self.robot_label} robot in the USD stage. "
                f"Checked: {self.config.get('prim_search_paths', [])}. "
                f"Available roots: {available}"
            )

        self.robot_prim_path = robot_prim_path
        self.articulation = world.scene.add(
            Articulation(
                prim_path=robot_prim_path,
                name=str(self.config.get("articulation_name", self.config.get("robot_type", "robot"))),
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

    def reset_runtime_state(self) -> None:
        self.left_runtime.last_arm_positions = None
        self.right_runtime.last_arm_positions = None
        self._smoothed_left_gripper = float(self.config["grippers"]["open_position"])
        self._smoothed_right_gripper = float(self.config["grippers"]["open_position"])

    def get_recording_vector(
        self,
        *,
        vector_config,
        current_joint_positions,
        commanded_action,
        teleop_targets=None,
    ) -> tuple[list[str], np.ndarray]:
        mode = getattr(vector_config, "mode", None)
        if mode is None and isinstance(vector_config, dict):
            mode = vector_config.get("mode", "articulation_joints")
        if mode != "named_groups":
            return super().get_recording_vector(
                vector_config=vector_config,
                current_joint_positions=current_joint_positions,
                commanded_action=commanded_action,
                teleop_targets=teleop_targets,
            )

        config_mapping = (
            vector_config.to_mapping() if hasattr(vector_config, "to_mapping") else dict(vector_config)
        )
        group_names = list(config_mapping.get("groups", []))
        joint_groups = self.config.get("recording", {}).get("joint_groups", {})
        if not group_names:
            raise ValueError("Recording vector config must define at least one group")

        state_source = np.asarray(current_joint_positions, dtype=float).reshape(-1)
        action_source = np.asarray(commanded_action.joint_positions, dtype=float).reshape(-1)
        use_action_source = config_mapping.get("key") == "action"

        names: list[str] = []
        values: list[float] = []
        for group_name in group_names:
            group = joint_groups.get(group_name)
            if group is None:
                raise KeyError(f"Unknown recording joint group '{group_name}'")

            source = str(group.get("source", "articulation"))
            if source == "articulation":
                joint_names = list(group.get("joints", []))
                vector = self.get_articulation_vector_by_joint_names(
                    joint_names,
                    action_source if use_action_source else state_source,
                )
                names.extend(joint_names)
                values.extend(vector.tolist())
                continue

            if source == "adapter" and group.get("value") == "normalized_gripper":
                side = str(group.get("side", "")).strip().lower()
                names.append(str(group.get("name", f"{side}_gripper")))
                raw_value = self._recording_gripper_source_value(
                    side=side,
                    source_positions=action_source if use_action_source else state_source,
                )
                values.append(self._normalize_gripper(raw_value))
                continue

            raise ValueError(f"Unsupported recording group definition for '{group_name}': {group}")

        return names, np.asarray(values, dtype=np.float32)

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
            arm_positions = self._limit_arm_step(
                runtime=runtime,
                arm_positions=arm_positions,
                limit_key=success_key.replace("_ik_success", "_ik_step_limited"),
            )
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

    @property
    def max_ik_joint_step_rad(self) -> float | None:
        value = self.config.get("safety", {}).get(
            "max_ik_joint_step_rad",
            self.config.get("ik", {}).get("max_ik_joint_step_rad", 0.04),
        )
        if value is None:
            return None
        return max(0.0, float(value))

    def _limit_arm_step(
        self,
        *,
        runtime: _ArmRuntime,
        arm_positions: np.ndarray,
        limit_key: str,
    ) -> np.ndarray:
        max_step = self.max_ik_joint_step_rad
        if max_step is None or max_step <= 0.0 or runtime.last_arm_positions is None:
            return arm_positions.copy()

        previous = np.asarray(runtime.last_arm_positions, dtype=float).reshape(-1)
        candidate = np.asarray(arm_positions, dtype=float).reshape(-1)
        if previous.size != candidate.size:
            return candidate.copy()

        delta = candidate - previous
        limited_delta = np.clip(delta, -max_step, max_step)
        if np.any(np.abs(delta - limited_delta) > 1e-9):
            self._diagnostics.counters[limit_key] = (
                self._diagnostics.counters.get(limit_key, 0) + 1
            )
        return previous + limited_delta

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

    def _recording_gripper_source_value(self, *, side: str, source_positions: np.ndarray) -> float:
        if side == "left":
            indices = self.left_gripper_indices
        elif side == "right":
            indices = self.right_gripper_indices
        else:
            raise ValueError(f"Unsupported gripper side '{side}'")
        if not indices:
            raise RuntimeError(f"Recording gripper indices for side '{side}' are not initialized")
        values = np.asarray(source_positions, dtype=float).reshape(-1)[indices]
        return float(np.mean(values))

    def _normalize_gripper(self, raw_value: float) -> float:
        open_position = float(self.config["grippers"]["open_position"])
        closed_position = float(self.config["grippers"]["closed_position"])
        denom = closed_position - open_position
        if abs(denom) < 1e-9:
            return 0.0
        normalized = (float(raw_value) - open_position) / denom
        return float(np.clip(normalized, 0.0, 1.0))
