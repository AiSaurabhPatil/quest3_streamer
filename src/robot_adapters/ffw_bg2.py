from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

from .acone import AconeAdapter


class FFWBG2Adapter(AconeAdapter):
    """FFW BG2 uses arm_base_link-rooted Lula descriptors with world-space teleop targets."""

    def __init__(self, config: dict, project_root: str):
        super().__init__(config, project_root)
        self.target_position_offset = np.asarray(
            self.ik_config.get("target_position_offset", [0.0, 0.0, 0.0]),
            dtype=float,
        ).reshape(3)
        self._diagnostics.details["target_position_offset"] = self.target_position_offset.tolist()

    def _compute_ik(self, *, solver, runtime, ee_target, warm_start, orientation):
        return solver.compute_inverse_kinematics(
            target_position=np.asarray(ee_target.position_xyz, dtype=float) + self.target_position_offset,
            target_orientation=orientation,
            frame_name=runtime.frame_name,
            warm_start=warm_start,
            position_tolerance=self.position_tolerance,
            orientation_tolerance=self.orientation_tolerance,
        )

    def configure_runtime_home_from_current_pose(self, runtime_config):
        current_positions = self.get_current_joint_positions()
        if current_positions is None or self.left_ik_solver is None or self.right_ik_solver is None:
            return runtime_config

        left_home = self._home_pose_from_fk(
            self.left_ik_solver,
            self.left_runtime.frame_name,
            np.asarray(current_positions, dtype=float)[self.left_arm_indices],
        )
        right_home = self._home_pose_from_fk(
            self.right_ik_solver,
            self.right_runtime.frame_name,
            np.asarray(current_positions, dtype=float)[self.right_arm_indices],
        )
        if left_home is None or right_home is None:
            return runtime_config

        left_position, left_orientation = left_home
        right_position, right_orientation = right_home
        center = (left_position + right_position) * 0.5
        runtime_config.robot_workspace_center = center
        runtime_config.left_arm_offset = left_position - center
        runtime_config.right_arm_offset = right_position - center
        runtime_config.left_arm_home_orientation = left_orientation
        runtime_config.right_arm_home_orientation = right_orientation
        self._diagnostics.details["dynamic_left_home"] = left_position.tolist()
        self._diagnostics.details["dynamic_right_home"] = right_position.tolist()
        self._diagnostics.details["dynamic_left_home_orientation"] = left_orientation.tolist()
        self._diagnostics.details["dynamic_right_home_orientation"] = right_orientation.tolist()
        print(f"[Init] FFW BG2 dynamic left home: {left_position}, orientation: {left_orientation}")
        print(f"[Init] FFW BG2 dynamic right home: {right_position}, orientation: {right_orientation}")
        return runtime_config

    def _home_pose_from_fk(self, solver, frame_name: str, joint_positions: np.ndarray):
        try:
            result = solver.compute_forward_kinematics(frame_name, joint_positions)
        except Exception as exc:
            self._diagnostics.details["dynamic_home_error"] = str(exc)
            return None
        if not isinstance(result, tuple) or len(result) != 2:
            return None
        root_position = np.asarray(result[0], dtype=float).reshape(-1)[:3]
        return (
            self._debug_fk_position_to_teleop_position(root_position),
            self._wxyz_from_fk_orientation(result[1]),
        )

    def _debug_fk_position_to_teleop_position(self, position: np.ndarray) -> np.ndarray:
        return np.asarray(position, dtype=float).reshape(3) - self.target_position_offset

    def _wxyz_from_fk_orientation(self, orientation) -> np.ndarray:
        orientation = np.asarray(orientation, dtype=float)
        if orientation.shape == (3, 3):
            quat_xyzw = R.from_matrix(orientation).as_quat()
            quat_wxyz = np.array(
                [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
                dtype=float,
            )
        else:
            quat_wxyz = orientation.reshape(4).astype(float)
        norm = np.linalg.norm(quat_wxyz)
        if norm <= 1e-9:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        quat_wxyz = quat_wxyz / norm
        if quat_wxyz[0] < 0.0:
            quat_wxyz *= -1.0
        return quat_wxyz
