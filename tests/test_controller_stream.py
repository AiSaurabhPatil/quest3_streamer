import os
import sys
import tempfile
import unittest

import numpy as np


TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.join(TESTS_DIR, "..")
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from teleop_core import (  # noqa: E402
    BimanualTeleopSession,
    ControllerAxes,
    ControllerButtons,
    ControllerPose,
    ControllerState,
    FrameTransform,
    TeleopSessionConfig,
)
from tools.controller_stream import (  # noqa: E402
    ControllerStreamFrame,
    controller_state_from_mapping,
    controller_state_to_mapping,
    load_controller_stream,
    replay_session_updates,
    write_controller_stream,
)


class ControllerStreamTests(unittest.TestCase):
    def test_controller_state_round_trip_preserves_pose_axes_and_buttons(self):
        state = ControllerState(
            hand="left",
            sequence=12,
            source_timestamp=3.5,
            receive_time_s=7.0,
            pose=ControllerPose(
                position_xyz=[1.0, 2.0, 3.0],
                orientation_xyzw=[0.0, 0.0, 0.0, 1.0],
            ),
            axes=ControllerAxes(trigger=0.6, squeeze=0.4, thumbstick_x=0.1, thumbstick_y=-0.2),
            buttons=ControllerButtons(primary=True, secondary=False, menu=True, stick_click=True),
            client_epoch_ms=1000.0,
            ingress_receive_epoch_ms=1010.0,
            remote_receive_epoch_ms=1020.0,
            ros_publish_epoch_ms=1030.0,
            control_receive_epoch_ms=1040.0,
            isaac_apply_epoch_ms=1050.0,
        )

        restored = controller_state_from_mapping(
            controller_state_to_mapping(state),
            receive_time_s=11.0,
        )

        self.assertEqual(restored.hand, "left")
        self.assertEqual(restored.sequence, 12)
        self.assertEqual(restored.source_timestamp, 3.5)
        self.assertEqual(restored.receive_time_s, 11.0)
        self.assertTrue(restored.buttons.primary)
        self.assertTrue(restored.buttons.menu)
        self.assertEqual(restored.client_epoch_ms, 1000.0)
        self.assertEqual(restored.isaac_apply_epoch_ms, 1050.0)
        np.testing.assert_allclose(restored.pose.position_xyz, [1.0, 2.0, 3.0])
        np.testing.assert_allclose(restored.pose.orientation_xyzw, [0.0, 0.0, 0.0, 1.0])

    def test_write_and_load_controller_stream_round_trip(self):
        frames = [
            ControllerStreamFrame(
                t=0.0,
                left=ControllerState(
                    hand="left",
                    pose=ControllerPose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
                ),
            ),
            ControllerStreamFrame(
                t=0.1,
                right=ControllerState(
                    hand="right",
                    axes=ControllerAxes(trigger=0.9),
                    buttons=ControllerButtons(primary=True),
                ),
            ),
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "sample.jsonl")
            write_controller_stream(path, frames)
            loaded = load_controller_stream(path)

        self.assertEqual(len(loaded), 2)
        self.assertIsNotNone(loaded[0].left)
        self.assertIsNone(loaded[0].right)
        self.assertTrue(loaded[1].right.buttons.primary)
        self.assertAlmostEqual(loaded[1].right.axes.trigger, 0.9)

    def test_replay_session_updates_drives_bimanual_session_from_recording(self):
        session = BimanualTeleopSession(
            TeleopSessionConfig(
                smoothing=0.0,
                calibration_samples=2,
                robot_workspace_center=[0.3, 0.0, 0.3],
                left_arm_offset=[0.0, 0.15, 0.0],
                right_arm_offset=[0.0, -0.15, 0.0],
                gripper_threshold=0.5,
            ),
            frame_transform=FrameTransform(np.eye(3), np.eye(3)),
        )
        frames = [
            ControllerStreamFrame(
                t=0.0,
                left=ControllerState(
                    hand="left",
                    pose=ControllerPose([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
                ),
                right=ControllerState(
                    hand="right",
                    pose=ControllerPose([0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
                ),
            ),
            ControllerStreamFrame(
                t=0.1,
                left=ControllerState(
                    hand="left",
                    pose=ControllerPose([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
                ),
                right=ControllerState(
                    hand="right",
                    pose=ControllerPose([0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
                ),
            ),
            ControllerStreamFrame(
                t=0.2,
                left=ControllerState(
                    hand="left",
                    pose=ControllerPose([1.2, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
                    axes=ControllerAxes(trigger=0.8),
                ),
                right=ControllerState(
                    hand="right",
                    pose=ControllerPose([0.0, 1.4, 0.0], [0.0, 0.0, 0.0, 1.0]),
                ),
            ),
        ]

        updates = replay_session_updates(frames, session)
        final = updates[-1]

        self.assertTrue(final.ready)
        self.assertTrue(final.targets.left_gripper.closed)
        np.testing.assert_allclose(final.targets.left_ee.position_xyz, [0.5, 0.15, 0.3])
        np.testing.assert_allclose(final.targets.right_ee.position_xyz, [0.3, 0.25, 0.3])


if __name__ == "__main__":
    unittest.main()
