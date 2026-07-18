import os
import sys
import unittest

import numpy as np


TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.join(TESTS_DIR, "..")
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from teleop_core import (  # noqa: E402
    DEFAULT_TOOL_ROTATION_CORRECTION,
    DEFAULT_VR_TO_ROBOT,
    FrameTransform,
)


class Phase7FrameTransformTests(unittest.TestCase):
    def test_default_position_transform_matches_expected_axis_mapping(self):
        transform = FrameTransform(DEFAULT_VR_TO_ROBOT, DEFAULT_TOOL_ROTATION_CORRECTION)

        mapped = transform.position_offset_to_robot(np.array([1.0, 2.0, 3.0]))

        np.testing.assert_allclose(mapped, [-3.0, -1.0, 2.0])

    def test_identity_orientation_maps_to_expected_robot_wxyz_quaternion(self):
        transform = FrameTransform(DEFAULT_VR_TO_ROBOT, DEFAULT_TOOL_ROTATION_CORRECTION)

        quaternion = transform.orientation_xyzw_to_robot_wxyz(
            np.array([0.0, 0.0, 0.0, 1.0])
        )

        np.testing.assert_allclose(np.abs(quaternion), [0.0, 1.0, 0.0, 0.0], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
