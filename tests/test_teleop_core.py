import os
import sys
import unittest

import numpy as np


TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.join(TESTS_DIR, "..")
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from robot_adapters import OpenArmAdapter, PandaAdapter  # noqa: E402
from isaac_backend import (  # noqa: E402
    CameraImagePublishers,
    CameraManager,
    CameraManagerConfig,
    IsaacApp,
    IsaacAppConfig,
    JointStatePublisher,
)
from quest_ingress import (  # noqa: E402
    QuestPacket,
    TransportMetricsTracker,
    TransportTimestamps,
    format_ros_frame_id,
    parse_ros_frame_id_metadata,
)
from teleop_core import (  # noqa: E402
    BimanualCalibration,
    BimanualTeleopSession,
    BimanualTeleopTargets,
    ControllerAxes,
    ControllerPose,
    ControllerState,
    DEFAULT_TOOL_ROTATION_CORRECTION,
    DEFAULT_VR_TO_ROBOT,
    EndEffectorTarget,
    FrameTransform,
    GripperTarget,
    OrientationSlerp,
    PositionEMA,
    SingleArmTeleopSession,
    SingleArmTeleopTargets,
    TargetSafety,
    TargetSafetyConfig,
    TeleopSessionConfig,
)


class ControllerStateTests(unittest.TestCase):
    def test_controller_pose_normalizes_shapes(self):
        pose = ControllerPose(
            position_xyz=[1.0, 2.0, 3.0],
            orientation_xyzw=[0.0, 0.0, 0.0, 1.0],
        )
        self.assertEqual(pose.position_xyz.shape, (3,))
        self.assertEqual(pose.orientation_xyzw.shape, (4,))

    def test_controller_state_reports_pose_age(self):
        state = ControllerState(hand="left", receive_time_s=5.0)
        self.assertEqual(state.age_s(7.5), 2.5)


class CalibrationTests(unittest.TestCase):
    def test_bimanual_calibration_completes_per_hand(self):
        calibration = BimanualCalibration(samples_required=2)
        left = ControllerState(
            hand="left",
            pose=ControllerPose([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        right = ControllerState(
            hand="right",
            pose=ControllerPose([0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )

        calibration.update(left, None)
        calibration.update(left, right)
        calibration.update(None, right)

        self.assertTrue(calibration.left.complete)
        self.assertTrue(calibration.right.complete)
        np.testing.assert_allclose(calibration.left.reference_position, [1.0, 0.0, 0.0])
        np.testing.assert_allclose(calibration.right.reference_position, [0.0, 1.0, 0.0])


class TransformAndFilterTests(unittest.TestCase):
    def test_frame_transform_outputs_expected_shapes(self):
        transform = FrameTransform(DEFAULT_VR_TO_ROBOT, DEFAULT_TOOL_ROTATION_CORRECTION)
        position = transform.position_offset_to_robot(np.array([1.0, 0.0, 0.0]))
        orientation = transform.orientation_xyzw_to_robot_wxyz(
            np.array([0.0, 0.0, 0.0, 1.0])
        )
        self.assertEqual(position.shape, (3,))
        self.assertEqual(orientation.shape, (4,))

    def test_position_ema_initializes_from_first_target(self):
        ema = PositionEMA(alpha=0.9)
        np.testing.assert_allclose(ema.update(np.array([1.0, 2.0, 3.0])), [1.0, 2.0, 3.0])

    def test_orientation_slerp_returns_normalized_quaternion(self):
        slerp = OrientationSlerp(alpha=0.9)
        quat = slerp.update(np.array([1.0, 0.0, 0.0, 0.0]))
        self.assertAlmostEqual(np.linalg.norm(quat), 1.0)


class RetargetingTests(unittest.TestCase):
    def test_end_effector_target_normalizes_shapes(self):
        target = EndEffectorTarget(
            position_xyz=[1.0, 2.0, 3.0],
            orientation_wxyz=[1.0, 0.0, 0.0, 0.0],
        )
        self.assertEqual(target.position_xyz.shape, (3,))
        self.assertEqual(target.orientation_wxyz.shape, (4,))

    def test_single_and_bimanual_targets_construct_without_ros(self):
        single = SingleArmTeleopTargets(
            ee_target=EndEffectorTarget(),
            gripper_target=GripperTarget(closed=True, analog_value=0.75),
        )
        bimanual = BimanualTeleopTargets(
            left_ee=EndEffectorTarget(),
            right_ee=EndEffectorTarget(),
            left_gripper=GripperTarget(closed=True),
            right_gripper=GripperTarget(closed=False),
        )
        self.assertTrue(single.gripper_target.closed)
        self.assertTrue(bimanual.left_gripper.closed)
        self.assertFalse(bimanual.right_gripper.closed)


class SessionTests(unittest.TestCase):
    def test_session_calibrates_then_generates_bimanual_targets(self):
        config = TeleopSessionConfig(
            robot_workspace_center=[0.3, 0.0, 0.3],
            left_arm_offset=[0.0, 0.15, 0.0],
            right_arm_offset=[0.0, -0.15, 0.0],
            smoothing=0.0,
            calibration_samples=2,
        )
        session = BimanualTeleopSession(
            config,
            frame_transform=FrameTransform(np.eye(3), np.eye(3)),
        )

        left_1 = ControllerState(
            hand="left",
            receive_time_s=1.0,
            source_timestamp=1.0,
            pose=ControllerPose([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        right_1 = ControllerState(
            hand="right",
            receive_time_s=1.0,
            source_timestamp=1.0,
            pose=ControllerPose([0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        initial = session.update({"left": left_1, "right": right_1}, now_s=1.0)
        self.assertFalse(initial.ready)

        left_2 = ControllerState(
            hand="left",
            receive_time_s=1.1,
            source_timestamp=1.1,
            pose=ControllerPose([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        right_2 = ControllerState(
            hand="right",
            receive_time_s=1.1,
            source_timestamp=1.1,
            pose=ControllerPose([0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        calibrated = session.update({"left": left_2, "right": right_2}, now_s=1.1)
        self.assertTrue(calibrated.ready)
        np.testing.assert_allclose(calibrated.targets.left_ee.position_xyz, [0.3, 0.15, 0.3])
        np.testing.assert_allclose(calibrated.targets.right_ee.position_xyz, [0.3, -0.15, 0.3])

        left_3 = ControllerState(
            hand="left",
            receive_time_s=1.2,
            source_timestamp=1.2,
            pose=ControllerPose([1.2, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            axes=ControllerAxes(trigger=0.8),
        )
        right_3 = ControllerState(
            hand="right",
            receive_time_s=1.2,
            source_timestamp=1.2,
            pose=ControllerPose([0.0, 1.4, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        updated = session.update({"left": left_3, "right": right_3}, now_s=1.2)

        np.testing.assert_allclose(updated.targets.left_ee.position_xyz, [0.5, 0.15, 0.3])
        np.testing.assert_allclose(updated.targets.right_ee.position_xyz, [0.3, 0.25, 0.3])
        np.testing.assert_allclose(updated.targets.left_ee.orientation_wxyz, [1.0, 0.0, 0.0, 0.0])
        self.assertTrue(updated.targets.left_gripper.closed)

    def test_session_hard_timeout_opens_gripper(self):
        config = TeleopSessionConfig(
            smoothing=0.0,
            calibration_samples=1,
            deadman_timeout_s=0.25,
            hard_timeout_s=0.5,
            gripper_threshold=0.5,
        )
        session = BimanualTeleopSession(
            config,
            frame_transform=FrameTransform(np.eye(3), np.eye(3)),
        )
        left = ControllerState(
            hand="left",
            receive_time_s=10.0,
            source_timestamp=10.0,
            pose=ControllerPose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        right = ControllerState(
            hand="right",
            receive_time_s=10.0,
            source_timestamp=10.0,
            pose=ControllerPose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            axes=ControllerAxes(trigger=0.9),
        )
        ready = session.update({"left": left, "right": right}, now_s=10.0)
        self.assertTrue(ready.ready)
        self.assertTrue(ready.targets.right_gripper.closed)

        stale = session.update({"left": left, "right": right}, now_s=10.7)
        self.assertFalse(stale.targets.right_gripper.closed)
        self.assertTrue(stale.right_state.hard_timeout_active)
        self.assertTrue(
            any("RIGHT controller hard timeout" in event.message for event in stale.events)
        )

    def test_single_arm_session_generates_targets_with_prediction(self):
        config = TeleopSessionConfig(
            robot_workspace_center=[0.5, 0.0, 0.4],
            smoothing=0.0,
            calibration_samples=1,
            enable_prediction=True,
            prediction_horizon_s=0.05,
        )
        session = SingleArmTeleopSession(
            config,
            hand="right",
            frame_transform=FrameTransform(np.eye(3), np.eye(3)),
        )

        first = ControllerState(
            hand="right",
            sequence=1,
            receive_time_s=1.0,
            source_timestamp=1.0,
            pose=ControllerPose([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        calibrated = session.update(first, now_s=1.0)
        self.assertTrue(calibrated.ready)

        second = ControllerState(
            hand="right",
            sequence=2,
            receive_time_s=1.1,
            source_timestamp=1.1,
            pose=ControllerPose([1.1, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        session.update(second, now_s=1.1)

        third = ControllerState(
            hand="right",
            sequence=3,
            receive_time_s=1.2,
            source_timestamp=1.2,
            pose=ControllerPose([1.2, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        updated = session.update(third, now_s=1.25)

        np.testing.assert_allclose(updated.targets.ee_target.position_xyz, [0.75, 0.0, 0.4])
        self.assertEqual(updated.hand_state.sequence, 3)


class SafetyTests(unittest.TestCase):
    def test_safety_rejects_large_jump(self):
        safety = TargetSafety(TargetSafetyConfig(max_translation_step_m=0.05))
        position, accepted = safety.apply_position(np.array([0.0, 0.0, 0.0]))
        self.assertTrue(accepted)
        np.testing.assert_allclose(position, [0.0, 0.0, 0.0])

        position, accepted = safety.apply_position(
            np.array([1.0, 0.0, 0.0]),
            previous_xyz=np.array([0.0, 0.0, 0.0]),
        )
        self.assertFalse(accepted)
        np.testing.assert_allclose(position, [0.0, 0.0, 0.0])


class AdapterConfigTests(unittest.TestCase):
    def test_openarm_adapter_loads_yaml_without_isaac_imports(self):
        adapter = OpenArmAdapter.from_yaml(
            os.path.join(PROJECT_ROOT, "config", "robots", "openarm.yaml"),
            project_root=PROJECT_ROOT,
        )
        self.assertTrue(adapter.usd_path.endswith("openarm_bimanual.usd"))
        self.assertGreaterEqual(len(adapter.get_camera_specs()), 3)
        np.testing.assert_allclose(adapter.left_workspace_offset, [0.0, 0.15, 0.0])

    def test_panda_adapter_exposes_workspace_and_home(self):
        adapter = PandaAdapter.from_yaml(
            os.path.join(PROJECT_ROOT, "config", "robots", "panda.yaml"),
            project_root=PROJECT_ROOT,
        )
        np.testing.assert_allclose(adapter.robot_home, [0.5, 0.0, 0.4])
        clamped = adapter.workspace_bounds.clamp(np.array([2.0, 0.0, -1.0]))
        np.testing.assert_allclose(clamped, [0.85, 0.0, 0.02])


class QuestIngressTests(unittest.TestCase):
    def test_packet_round_trip_preserves_transport_timestamps(self):
        packet = QuestPacket.from_mapping(
            {
                "schema_version": 1,
                "sequence": 42,
                "timestamp": 12.5,
                "client_epoch_ms": 1000.0,
                "transport": {
                    "ingress_receive_epoch_ms": 1015.0,
                    "remote_receive_epoch_ms": 1040.0,
                },
                "controllers": {
                    "left": {
                        "position": {"x": 1.0, "y": 2.0, "z": 3.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                        "trigger": 0.8,
                        "button_a_x": True,
                    }
                },
            }
        )

        self.assertEqual(packet.sequence, 42)
        self.assertEqual(packet.transport.client_epoch_ms, 1000.0)
        self.assertEqual(packet.transport.ingress_receive_epoch_ms, 1015.0)
        self.assertEqual(packet.controllers["left"].position_xyz, (1.0, 2.0, 3.0))

        payload = packet.to_mapping()
        self.assertEqual(payload["transport"]["remote_receive_epoch_ms"], 1040.0)
        self.assertTrue(payload["controllers"]["left"]["button_a_x"])

    def test_ros_frame_id_metadata_round_trip(self):
        packet = QuestPacket(
            sequence=7,
            timestamp=123.4567,
            transport=TransportTimestamps(
                client_epoch_ms=1000.0,
                ingress_receive_epoch_ms=1010.0,
                remote_receive_epoch_ms=1020.0,
            ),
        )

        frame_id = format_ros_frame_id(packet, ros_publish_epoch_ms=1030.0)
        base_frame, metadata = parse_ros_frame_id_metadata(frame_id)

        self.assertEqual(base_frame, "quest_world")
        self.assertEqual(metadata["seq"], 7)
        self.assertAlmostEqual(metadata["ts"], 123.457)
        self.assertEqual(metadata["client"], 1000.0)
        self.assertEqual(metadata["ros"], 1030.0)

    def test_transport_metrics_tracker_reports_latency_percentiles(self):
        tracker = TransportMetricsTracker(label="remote-receiver", log_period_s=1.0, window_size=8)
        packet_1 = QuestPacket(
            sequence=1,
            transport=TransportTimestamps(
                client_epoch_ms=1000.0,
                ingress_receive_epoch_ms=1010.0,
            ),
        )
        packet_2 = QuestPacket(
            sequence=3,
            transport=TransportTimestamps(
                client_epoch_ms=1020.0,
                ingress_receive_epoch_ms=1030.0,
            ),
        )

        tracker.record(packet_1, payload_size_bytes=200, receive_monotonic_s=1.0, receive_epoch_ms=1040.0)
        tracker.record(packet_2, payload_size_bytes=220, receive_monotonic_s=1.2, receive_epoch_ms=1060.0)
        snapshot = tracker.snapshot(now_monotonic_s=2.2)

        self.assertEqual(snapshot.dropped_packets, 1)
        self.assertGreater(snapshot.packet_rate_hz, 0.0)
        self.assertAlmostEqual(snapshot.age_p50_ms, 40.0)
        self.assertAlmostEqual(snapshot.vpn_hop_p50_ms, 30.0)


class IsaacBackendImportTests(unittest.TestCase):
    def test_backend_modules_import_without_isaac_runtime(self):
        self.assertIs(IsaacApp, IsaacApp)
        self.assertIs(IsaacAppConfig, IsaacAppConfig)
        self.assertIs(CameraManager, CameraManager)
        self.assertIs(CameraManagerConfig, CameraManagerConfig)
        self.assertIs(CameraImagePublishers, CameraImagePublishers)
        self.assertIs(JointStatePublisher, JointStatePublisher)


if __name__ == "__main__":
    unittest.main()
