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

    def position_offset_to_robot_at_heading(
        self,
        xr_offset_xyz: np.ndarray,
        reference_orientation_xyzw: np.ndarray,
    ) -> np.ndarray:
        """Map room-space motion relative to the controller heading at engagement."""
        xr_offset_xyz = np.asarray(xr_offset_xyz, dtype=float).reshape(3)
        heading_basis = self._heading_basis(reference_orientation_xyzw)
        canonical_xr_offset = heading_basis.T @ xr_offset_xyz
        return self.matrix_vr_to_robot @ canonical_xr_offset

    def relative_orientation_to_robot_wxyz(
        self,
        current_orientation_xyzw: np.ndarray,
        reference_orientation_xyzw: np.ndarray,
        anchor_orientation_wxyz: np.ndarray,
    ) -> np.ndarray:
        """Apply a heading-relative controller rotation to an anchored EE pose."""
        current = R.from_quat(
            np.asarray(current_orientation_xyzw, dtype=float).reshape(4)
        )
        reference = R.from_quat(
            np.asarray(reference_orientation_xyzw, dtype=float).reshape(4)
        )
        heading_basis = self._heading_basis(reference_orientation_xyzw)

        room_delta = (current * reference.inv()).as_matrix()
        heading_delta = heading_basis.T @ room_delta @ heading_basis
        robot_delta = (
            self.matrix_vr_to_robot
            @ heading_delta
            @ self.matrix_vr_to_robot.T
        )

        anchor_wxyz = np.asarray(anchor_orientation_wxyz, dtype=float).reshape(4)
        anchor = R.from_quat(
            [anchor_wxyz[1], anchor_wxyz[2], anchor_wxyz[3], anchor_wxyz[0]]
        )
        quat_xyzw = (R.from_matrix(robot_delta) * anchor).as_quat()
        quat_wxyz = np.array(
            [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
            dtype=float,
        )
        if quat_wxyz[0] < 0.0:
            quat_wxyz *= -1.0
        return quat_wxyz / np.linalg.norm(quat_wxyz)

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

    @staticmethod
    def _heading_basis(reference_orientation_xyzw: np.ndarray) -> np.ndarray:
        """Return canonical XR right/up/back axes expressed in room space."""
        reference = R.from_quat(
            np.asarray(reference_orientation_xyzw, dtype=float).reshape(4)
        )
        forward = reference.apply([0.0, 0.0, -1.0])
        forward[1] = 0.0
        forward_norm = np.linalg.norm(forward)
        if forward_norm <= 1e-9:
            return np.eye(3, dtype=float)

        forward /= forward_norm
        up = np.array([0.0, 1.0, 0.0], dtype=float)
        right = np.cross(forward, up)
        right /= np.linalg.norm(right)
        return np.column_stack((right, up, -forward))
