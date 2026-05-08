from __future__ import annotations

import numpy as np

from .openarm import OpenArmAdapter


class AconeAdapter(OpenArmAdapter):
    """Bimanual custom-URDF adapter for the AC One robot."""

    def __init__(self, config: dict, project_root: str):
        super().__init__(config, project_root)
        self.ik_config = dict(self.config.get("ik", {}))
        self.orientation_mode = str(
            self.ik_config.get("orientation_mode", "position_only")
        ).strip()
        self.position_tolerance = _optional_float(self.ik_config.get("position_tolerance"))
        self.orientation_tolerance = _optional_float(self.ik_config.get("orientation_tolerance"))
        self.orientation_fallback_to_position = bool(
            self.ik_config.get("orientation_fallback_to_position", True)
        )
        self._diagnostics.details["orientation_mode"] = self.orientation_mode
        self._diagnostics.details["orientation_fallback_to_position"] = (
            self.orientation_fallback_to_position
        )
        self._diagnostics.counters.update(
            {
                "left_orientation_fallback": 0,
                "right_orientation_fallback": 0,
            }
        )

    def initialize_joint_mappings(self) -> None:
        super().initialize_joint_mappings()
        self._apply_gripper_drive_overrides()

    def _apply_arm_ik(
        self,
        solver,
        runtime,
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
        orientation = self._target_orientation(ee_target)

        actions, success = self._compute_ik(
            solver=solver,
            runtime=runtime,
            ee_target=ee_target,
            warm_start=warm_start,
            orientation=orientation,
        )
        if not success and orientation is not None and self.orientation_fallback_to_position:
            actions, success = self._compute_ik(
                solver=solver,
                runtime=runtime,
                ee_target=ee_target,
                warm_start=warm_start,
                orientation=None,
            )
            if success:
                self._diagnostics.counters[
                    success_key.replace("_ik_success", "_orientation_fallback")
                ] += 1

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

    def _compute_ik(self, *, solver, runtime, ee_target, warm_start, orientation):
        return solver.compute_inverse_kinematics(
            target_position=ee_target.position_xyz,
            target_orientation=orientation,
            frame_name=runtime.frame_name,
            warm_start=warm_start,
            position_tolerance=self.position_tolerance,
            orientation_tolerance=self.orientation_tolerance,
        )

    def _target_orientation(self, ee_target):
        if self.orientation_mode in {"position_only", "position-only", "none"}:
            return None
        return ee_target.orientation_wxyz

    def _apply_gripper_drive_overrides(self) -> None:
        drive_config = self.config.get("grippers", {}).get("drive", {})
        if not drive_config or self.articulation is None:
            return

        try:
            from pxr import UsdPhysics
        except Exception as exc:
            self._diagnostics.details["gripper_drive_error"] = str(exc)
            return

        stage = self.articulation.prim.GetStage()
        gripper_joints = set(self.config["grippers"]["left_joints"]) | set(
            self.config["grippers"]["right_joints"]
        )
        for prim in stage.Traverse():
            if prim.GetName() not in gripper_joints:
                continue
            drive = UsdPhysics.DriveAPI.Apply(prim, "linear")
            if "stiffness" in drive_config:
                drive.CreateStiffnessAttr(float(drive_config["stiffness"]))
            if "damping" in drive_config:
                drive.CreateDampingAttr(float(drive_config["damping"]))
            if "max_force" in drive_config:
                drive.CreateMaxForceAttr(float(drive_config["max_force"]))


def _optional_float(value) -> float | None:
    if value in (None, ""):
        return None
    return float(value)
