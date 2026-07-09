from __future__ import annotations

import numpy as np

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
        # FFW has always derived its teleop home pose from forward kinematics
        # (its workspace_center is nominal only). Ensure the base
        # configure_runtime_home_from_current_pose runs even if the config
        # omits configure_home_from_fk, unless explicitly disabled.
        if "configure_home_from_fk" not in self.ik_config:
            self.ik_config["configure_home_from_fk"] = True

    def _compute_ik(self, *, solver, runtime, ee_target, warm_start, orientation):
        return solver.compute_inverse_kinematics(
            target_position=np.asarray(ee_target.position_xyz, dtype=float) + self.target_position_offset,
            target_orientation=orientation,
            frame_name=runtime.frame_name,
            warm_start=warm_start,
            position_tolerance=self.position_tolerance,
            orientation_tolerance=self.orientation_tolerance,
        )

    def _fk_position_to_teleop_position(self, position: np.ndarray) -> np.ndarray:
        # FFW's Lula root_link (arm_base_link) is offset from the stage origin
        # by target_position_offset, so the FK world position must be shifted
        # back into the teleop/stage frame. This is the inverse of the offset
        # applied in _compute_ik above, keeping FK home pose and IK targets in
        # the same coordinate system.
        return np.asarray(position, dtype=float).reshape(3) - self.target_position_offset
