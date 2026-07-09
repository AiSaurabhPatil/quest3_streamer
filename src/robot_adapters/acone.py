from __future__ import annotations

import numpy as np

from .openarm import OpenArmAdapter


class AconeAdapter(OpenArmAdapter):
    """Bimanual custom-URDF adapter for the AC One robot."""

    def __init__(self, config: dict, project_root: str):
        super().__init__(config, project_root)
        # IK config (ik_config, orientation_mode, tolerances, fallback) is now
        # parsed by the OpenArmAdapter base. Acone historically defaulted to
        # position-only IK, so preserve that default when the config omits an
        # explicit orientation_mode.
        if "orientation_mode" not in self.ik_config:
            self.orientation_mode = "position_only"
            self._diagnostics.details["orientation_mode"] = self.orientation_mode
        # orientation_fallback_to_position defaults to True for Acone (the base
        # default is False) so a full-pose miss degrades to position-only.
        if "orientation_fallback_to_position" not in self.ik_config:
            self.orientation_fallback_to_position = True
            self._diagnostics.details["orientation_fallback_to_position"] = True

    def initialize_joint_mappings(self) -> None:
        super().initialize_joint_mappings()
        self._apply_gripper_drive_overrides()

    def _solve_arm_ik(
        self,
        solver,
        runtime,
        indices: list[int],
        ee_target,
    ):
        """Acone-specific IK worker: tries full pose IK, then falls back to
        position-only if configured. Thread-safe pure result; counters and the
        shared command vector are updated on the control thread by _scatter_arm.
        """
        if solver is None or not ee_target.valid:
            return self._ArmIKResult(
                success=False,
                step_limited=False,
                arm_positions=None,
                fallback=runtime.last_arm_positions,
                indices=list(indices),
                orientation_fallback=False,
            )

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
        orientation_fallback = False
        if not success and orientation is not None and self.orientation_fallback_to_position:
            actions, success = self._compute_ik(
                solver=solver,
                runtime=runtime,
                ee_target=ee_target,
                warm_start=warm_start,
                orientation=None,
            )
            orientation_fallback = success

        if not success:
            return self._ArmIKResult(
                success=False,
                step_limited=False,
                arm_positions=None,
                fallback=runtime.last_arm_positions,
                indices=list(indices),
                orientation_fallback=False,
            )

        arm_positions = np.asarray(actions, dtype=float).reshape(-1)[: len(indices)]
        limited, step_limited = self._limit_arm_step_pure(
            runtime=runtime,
            arm_positions=arm_positions,
        )
        runtime.last_arm_positions = limited.copy()
        return self._ArmIKResult(
            success=True,
            step_limited=step_limited,
            arm_positions=limited,
            fallback=None,
            indices=list(indices),
            orientation_fallback=orientation_fallback,
        )

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
