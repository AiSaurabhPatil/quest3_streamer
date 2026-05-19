import os
import sys
import unittest

import numpy as np


TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.join(TESTS_DIR, "..")
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from robot_adapters import AconeAdapter, OpenArmAdapter, PandaAdapter  # noqa: E402
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

    def test_bimanual_orientation_is_relative_to_calibration_pose(self):
        config = TeleopSessionConfig(
            smoothing=0.0,
            calibration_samples=1,
        )
        session = BimanualTeleopSession(
            config,
            frame_transform=FrameTransform(np.eye(3), np.eye(3)),
        )
        reference_orientation = [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)]
        left = ControllerState(
            hand="left",
            receive_time_s=1.0,
            source_timestamp=1.0,
            pose=ControllerPose([0.0, 0.0, 0.0], reference_orientation),
        )
        right = ControllerState(
            hand="right",
            receive_time_s=1.0,
            source_timestamp=1.0,
            pose=ControllerPose([0.0, 0.0, 0.0], reference_orientation),
        )
        calibrated = session.update({"left": left, "right": right}, now_s=1.0)

        self.assertTrue(calibrated.ready)
        np.testing.assert_allclose(
            calibrated.targets.left_ee.orientation_wxyz,
            [1.0, 0.0, 0.0, 0.0],
            atol=1e-7,
        )

        updated = session.update({"left": left, "right": right}, now_s=1.1)
        np.testing.assert_allclose(
            updated.targets.left_ee.orientation_wxyz,
            [1.0, 0.0, 0.0, 0.0],
            atol=1e-7,
        )

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

    def test_session_reset_preserves_calibration_when_requested(self):
        config = TeleopSessionConfig(
            robot_workspace_center=[0.3, 0.0, 0.3],
            left_arm_offset=[0.0, 0.15, 0.0],
            right_arm_offset=[0.0, -0.15, 0.0],
            smoothing=0.0,
            calibration_samples=1,
        )
        session = BimanualTeleopSession(
            config,
            frame_transform=FrameTransform(np.eye(3), np.eye(3)),
        )

        left = ControllerState(
            hand="left",
            receive_time_s=1.0,
            source_timestamp=1.0,
            pose=ControllerPose([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        right = ControllerState(
            hand="right",
            receive_time_s=1.0,
            source_timestamp=1.0,
            pose=ControllerPose([0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        session.update({"left": left, "right": right}, now_s=1.0)

        left_reference = session.left.reference_position.copy()
        right_reference = session.right.reference_position.copy()

        session.reset(preserve_calibration=True)

        self.assertTrue(session.left.calibrated)
        self.assertTrue(session.right.calibrated)
        np.testing.assert_allclose(session.left.reference_position, left_reference)
        np.testing.assert_allclose(session.right.reference_position, right_reference)
        np.testing.assert_allclose(session.left.target_pos, [0.3, 0.15, 0.3])
        np.testing.assert_allclose(session.right.target_pos, [0.3, -0.15, 0.3])

    def test_session_reset_clears_calibration_by_default(self):
        config = TeleopSessionConfig(
            smoothing=0.0,
            calibration_samples=1,
        )
        session = BimanualTeleopSession(
            config,
            frame_transform=FrameTransform(np.eye(3), np.eye(3)),
        )

        left = ControllerState(
            hand="left",
            receive_time_s=1.0,
            source_timestamp=1.0,
            pose=ControllerPose([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        right = ControllerState(
            hand="right",
            receive_time_s=1.0,
            source_timestamp=1.0,
            pose=ControllerPose([0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        session.update({"left": left, "right": right}, now_s=1.0)

        session.reset()

        self.assertFalse(session.left.calibrated)
        self.assertFalse(session.right.calibrated)
        self.assertIsNone(session.left.reference_position)
        self.assertIsNone(session.right.reference_position)
        self.assertEqual(session.calibration.status.left_samples, 0)
        self.assertEqual(session.calibration.status.right_samples, 0)

    def test_single_arm_session_generates_targets_with_prediction(self):
        config = TeleopSessionConfig(
            robot_workspace_center=[0.5, 0.0, 0.4],
            smoothing=0.0,
            calibration_samples=1,
            enable_prediction=True,
            prediction_horizon_s=0.05,
            max_target_velocity_mps=None,
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

    def test_fresh_pose_after_soft_stale_gap_is_blended(self):
        config = TeleopSessionConfig(
            robot_workspace_center=[0.5, 0.0, 0.4],
            smoothing=0.0,
            calibration_samples=1,
            deadman_timeout_s=0.25,
            hard_timeout_s=1.0,
            stale_recovery_alpha=0.25,
            max_target_velocity_mps=None,
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
        session.update(first, now_s=1.0)

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
        session.update(third, now_s=1.2)

        stale = session.update(third, now_s=1.5)

        self.assertTrue(stale.hand_state.stale)
        self.assertFalse(stale.hand_state.hard_timeout_active)

        fourth = ControllerState(
            hand="right",
            sequence=4,
            receive_time_s=1.5,
            source_timestamp=1.5,
            pose=ControllerPose([1.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        recovered = session.update(fourth, now_s=1.5)

        self.assertFalse(recovered.hand_state.stale)
        np.testing.assert_allclose(recovered.targets.ee_target.position_xyz, [0.775, 0.0, 0.4])

    def test_target_velocity_limit_slows_large_controller_motion(self):
        config = TeleopSessionConfig(
            robot_workspace_center=[0.5, 0.0, 0.4],
            smoothing=0.0,
            calibration_samples=1,
            max_target_velocity_mps=0.2,
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
            pose=ControllerPose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        session.update(first, now_s=1.0)

        second = ControllerState(
            hand="right",
            sequence=2,
            receive_time_s=1.1,
            source_timestamp=1.1,
            pose=ControllerPose([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        session.update(second, now_s=1.1)

        third = ControllerState(
            hand="right",
            sequence=3,
            receive_time_s=1.2,
            source_timestamp=1.2,
            pose=ControllerPose([2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        )
        limited = session.update(third, now_s=1.2)

        np.testing.assert_allclose(limited.targets.ee_target.position_xyz, [1.52, 0.0, 0.4])


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
    def test_acone_adapter_exposes_bimanual_offsets_and_gripper_settings(self):
        adapter = AconeAdapter.from_mapping(
            {
                "robot_type": "acone",
                "display_name": "AC One",
                "usd": "assets/acone.usd",
                "urdf": "assets/acone.urdf",
                "left_arm_config": "robot_configs/acone_config/left_arm",
                "right_arm_config": "robot_configs/acone_config/right_arm",
                "left_arm": {
                    "frame_name": "left_link6",
                    "joints": [
                        "left_joint1",
                        "left_joint2",
                        "left_joint3",
                        "left_joint4",
                        "left_joint5",
                        "left_joint6",
                    ],
                    "preferred_config": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "workspace_offset": [0.0, 0.25, 0.0],
                },
                "right_arm": {
                    "frame_name": "right_link16",
                    "joints": [
                        "right_joint11",
                        "right_joint12",
                        "right_joint13",
                        "right_joint14",
                        "right_joint15",
                        "right_joint16",
                    ],
                    "preferred_config": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "workspace_offset": [0.0, -0.25, 0.0],
                },
                "grippers": {
                    "open_position": 0.044,
                    "closed_position": 0.0,
                    "speed": 0.003,
                    "threshold": 0.5,
                    "left_joints": ["left_joint7", "left_joint8"],
                    "right_joints": ["right_joint17", "right_joint18"],
                },
                "ik": {
                    "orientation_mode": "position_only",
                    "position_tolerance": 0.015,
                },
            },
            project_root=PROJECT_ROOT,
        )
        self.assertTrue(adapter.usd_path.endswith("assets/acone.usd"))
        self.assertEqual(adapter.robot_label, "AC One")
        np.testing.assert_allclose(adapter.left_workspace_offset, [0.0, 0.25, 0.0])
        np.testing.assert_allclose(adapter.right_workspace_offset, [0.0, -0.25, 0.0])
        self.assertAlmostEqual(adapter.gripper_threshold, 0.5)
        self.assertEqual(adapter.orientation_mode, "position_only")
        self.assertIsNone(adapter._target_orientation(EndEffectorTarget()))

    def test_acone_adapter_falls_back_to_position_ik_when_orientation_fails(self):
        class FallbackSolver:
            def __init__(self):
                self.orientations = []

            def compute_inverse_kinematics(self, **kwargs):
                self.orientations.append(kwargs["target_orientation"])
                if kwargs["target_orientation"] is not None:
                    return np.zeros(6), False
                return np.arange(6, dtype=float), True

        adapter = AconeAdapter.from_mapping(
            {
                "robot_type": "acone",
                "usd": "assets/acone.usd",
                "urdf": "assets/acone.urdf",
                "left_arm_config": "robot_configs/acone_config/left_arm",
                "right_arm_config": "robot_configs/acone_config/right_arm",
                "left_arm": {
                    "frame_name": "left_link6",
                    "joints": [
                        "left_joint1",
                        "left_joint2",
                        "left_joint3",
                        "left_joint4",
                        "left_joint5",
                        "left_joint6",
                    ],
                    "preferred_config": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                },
                "right_arm": {
                    "frame_name": "right_link16",
                    "joints": [
                        "right_joint11",
                        "right_joint12",
                        "right_joint13",
                        "right_joint14",
                        "right_joint15",
                        "right_joint16",
                    ],
                    "preferred_config": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                },
                "grippers": {
                    "open_position": 0.044,
                    "closed_position": 0.0,
                    "speed": 0.003,
                    "left_joints": ["left_joint7", "left_joint8"],
                    "right_joints": ["right_joint17", "right_joint18"],
                },
                "ik": {
                    "orientation_mode": "full_pose",
                    "orientation_fallback_to_position": True,
                },
            },
            project_root=PROJECT_ROOT,
        )
        solver = FallbackSolver()
        target_positions = np.zeros(6, dtype=float)
        adapter._apply_arm_ik(
            solver=solver,
            runtime=adapter.left_runtime,
            indices=[0, 1, 2, 3, 4, 5],
            ee_target=EndEffectorTarget(
                position_xyz=[0.1, 0.2, 0.3],
                orientation_wxyz=[0.0, 1.0, 0.0, 0.0],
            ),
            target_positions=target_positions,
            success_key="left_ik_success",
            fail_key="left_ik_fail",
        )

        self.assertEqual(len(solver.orientations), 2)
        self.assertIsNone(solver.orientations[1])
        np.testing.assert_allclose(target_positions, np.arange(6, dtype=float))
        diagnostics = adapter.get_diagnostics()
        self.assertEqual(diagnostics.counters["left_ik_success"], 1)
        self.assertEqual(diagnostics.counters["left_ik_fail"], 0)
        self.assertEqual(diagnostics.counters["left_orientation_fallback"], 1)

    def test_openarm_adapter_limits_large_ik_joint_steps(self):
        adapter = OpenArmAdapter.from_mapping(
            {
                "robot_type": "openarm",
                "usd": "assets/openarm.usd",
                "urdf": "assets/openarm.urdf",
                "left_arm_config": "robot_configs/openarm_config/left_arm",
                "right_arm_config": "robot_configs/openarm_config/right_arm",
                "left_arm": {
                    "frame_name": "left_hand",
                    "joints": ["left_joint1", "left_joint2", "left_joint3"],
                    "preferred_config": [0.0, 0.0, 0.0],
                },
                "right_arm": {
                    "frame_name": "right_hand",
                    "joints": ["right_joint1", "right_joint2", "right_joint3"],
                    "preferred_config": [0.0, 0.0, 0.0],
                },
                "grippers": {
                    "open_position": 0.044,
                    "closed_position": 0.0,
                    "speed": 0.003,
                    "left_joints": ["left_finger1"],
                    "right_joints": ["right_finger1"],
                },
                "ik": {"max_ik_joint_step_rad": 0.05},
            },
            project_root=PROJECT_ROOT,
        )
        adapter.left_runtime.last_arm_positions = np.zeros(3, dtype=float)

        limited = adapter._limit_arm_step(
            runtime=adapter.left_runtime,
            arm_positions=np.array([1.0, -1.0, 0.02]),
            limit_key="left_ik_step_limited",
        )

        np.testing.assert_allclose(limited, [0.05, -0.05, 0.02])
        self.assertEqual(adapter.get_diagnostics().counters["left_ik_step_limited"], 1)

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

    def test_isaac_app_does_not_forward_launcher_args_to_kit(self):
        import types

        captured_argv = []
        enabled_extensions = []

        class FakeSimulationApp:
            def __init__(self, *_args, **_kwargs):
                captured_argv.append(list(sys.argv))

            def update(self):
                pass

            def set_setting(self, *_args):
                pass

            def close(self):
                pass

        omni_mod = types.ModuleType("omni")
        isaac_mod = types.ModuleType("omni.isaac")
        kit_mod = types.ModuleType("omni.isaac.kit")
        core_mod = types.ModuleType("omni.isaac.core")
        utils_mod = types.ModuleType("omni.isaac.core.utils")
        extensions_mod = types.ModuleType("omni.isaac.core.utils.extensions")
        kit_mod.SimulationApp = FakeSimulationApp
        extensions_mod.enable_extension = enabled_extensions.append
        omni_mod.isaac = isaac_mod
        isaac_mod.kit = kit_mod
        isaac_mod.core = core_mod
        core_mod.utils = utils_mod
        utils_mod.extensions = extensions_mod

        module_names = (
            "omni",
            "omni.isaac",
            "omni.isaac.kit",
            "omni.isaac.core",
            "omni.isaac.core.utils",
            "omni.isaac.core.utils.extensions",
        )
        previous_modules = {name: sys.modules.get(name) for name in module_names}
        original_argv = sys.argv
        try:
            for module in (
                omni_mod,
                isaac_mod,
                kit_mod,
                core_mod,
                utils_mod,
                extensions_mod,
            ):
                sys.modules[module.__name__] = module
            sys.argv = ["teleop", "--config", "config.yaml", "--robot", "acone", "--webrtc"]
            IsaacApp({"webrtc_streaming": True, "simulation": {"headless": True}}).start()
        finally:
            sys.argv = original_argv
            for name, module in previous_modules.items():
                if module is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = module

        self.assertEqual(captured_argv, [["teleop"]])
        self.assertIn("omni.isaac.ros2_bridge", enabled_extensions)
        self.assertIn("omni.services.livestream.nvcf", enabled_extensions)


if __name__ == "__main__":
    unittest.main()
