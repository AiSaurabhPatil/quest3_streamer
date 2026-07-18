import os
import sys
import unittest

import numpy as np


TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.join(TESTS_DIR, "..")
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from teleop_core import VelocityLimiter, slerp_quat_wxyz  # noqa: E402


class Phase7FilterTests(unittest.TestCase):
    def test_slerp_handles_antipodal_quaternions_without_flipping(self):
        current = np.array([1.0, 0.0, 0.0, 0.0])
        target = np.array([-1.0, 0.0, 0.0, 0.0])

        blended = slerp_quat_wxyz(current, target, 0.5)

        np.testing.assert_allclose(blended, [1.0, 0.0, 0.0, 0.0])

    def test_velocity_limiter_caps_translation_step(self):
        limiter = VelocityLimiter(max_velocity_mps=0.5)

        limited = limiter.limit(
            current_xyz=np.array([0.0, 0.0, 0.0]),
            target_xyz=np.array([1.0, 0.0, 0.0]),
            dt_s=0.2,
        )

        np.testing.assert_allclose(limited, [0.1, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
