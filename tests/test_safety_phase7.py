import os
import sys
import unittest


TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.join(TESTS_DIR, "..")
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from teleop_core import ControllerPose, ControllerState, TargetSafety, TargetSafetyConfig  # noqa: E402


class Phase7SafetyTests(unittest.TestCase):
    def test_stale_detection_respects_timeout(self):
        safety = TargetSafety(TargetSafetyConfig(stale_timeout_s=0.25))
        state = ControllerState(
            hand="left",
            receive_time_s=10.0,
            pose=ControllerPose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )

        self.assertFalse(safety.is_state_stale(state, now_s=10.2))
        self.assertTrue(safety.is_state_stale(state, now_s=10.3))


if __name__ == "__main__":
    unittest.main()
