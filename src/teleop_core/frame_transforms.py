import numpy as np
from scipy.spatial.transform import Rotation as R


DEFAULT_VR_TO_ROBOT = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=float,
)
DEFAULT_TOOL_ROTATION_CORRECTION = R.from_euler("x", 180, degrees=True).as_matrix()


class FrameTransform:
    def __init__(
        self,
        matrix_vr_to_robot: np.ndarray,
        tool_rotation_correction: np.ndarray,
    ):
        self.matrix_vr_to_robot = np.asarray(matrix_vr_to_robot, dtype=float).reshape(3, 3)
        self.tool_rotation_correction = np.asarray(
            tool_rotation_correction,
            dtype=float,
        ).reshape(3, 3)

    def position_offset_to_robot(self, xr_offset_xyz: np.ndarray) -> np.ndarray:
        xr_offset_xyz = np.asarray(xr_offset_xyz, dtype=float).reshape(3)
        return self.matrix_vr_to_robot @ xr_offset_xyz

    def orientation_xyzw_to_robot_wxyz(self, xr_quat_xyzw: np.ndarray) -> np.ndarray:
        xr_quat_xyzw = np.asarray(xr_quat_xyzw, dtype=float).reshape(4)
        xr_rotation = R.from_quat(xr_quat_xyzw)
        robot_matrix = (
            self.matrix_vr_to_robot
            @ xr_rotation.as_matrix()
            @ self.matrix_vr_to_robot.T
        )
        robot_matrix = robot_matrix @ self.tool_rotation_correction
        quat_xyzw = R.from_matrix(robot_matrix).as_quat()
        return np.array(
            [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
            dtype=float,
        )
