import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.join(TESTS_DIR, "..")
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.config_loader import load_runtime_config  # noqa: E402
from src.recording import ButtonEdgeMapper, LeRobotEpisodeRecorder, RecordingConfig, build_recording_schema  # noqa: E402
from src.recording.worker_process import (  # noqa: E402
    _is_recreatable_empty_dataset_root,
    _supported_kwargs,
    _validate_existing_dataset_root,
)
from src.robot_adapters import OpenArmAdapter, RobotAction  # noqa: E402
from src.teleop_core import ControllerButtons, ControllerState  # noqa: E402


class RecordingTests(unittest.TestCase):
    def test_recording_config_resolves_root_and_defaults(self):
        config = RecordingConfig.from_mapping(
            {
                "enabled": True,
                "root": "datasets/test",
            },
            project_root=PROJECT_ROOT,
        )

        self.assertTrue(config.enabled)
        self.assertEqual(config.dataset_format, "v3.0")
        self.assertEqual(config.root, os.path.join(PROJECT_ROOT, "datasets/test"))
        self.assertEqual(config.buttons.save_episode, "left_primary")
        self.assertEqual(config.buttons.start_episode, "left_secondary")
        self.assertEqual(config.cameras.resolution, (224, 224))

    def test_button_edge_mapper_only_emits_rising_edges(self):
        mapper = ButtonEdgeMapper(RecordingConfig.from_mapping({}, project_root=PROJECT_ROOT).buttons)

        left_pressed = ControllerState(
            hand="left",
            buttons=ControllerButtons(primary=True, secondary=True),
        )
        right_pressed = ControllerState(hand="right", buttons=ControllerButtons(secondary=True))

        first = mapper.update({"left": left_pressed, "right": right_pressed})
        second = mapper.update({"left": left_pressed, "right": right_pressed})
        released = mapper.update(
            {
                "left": ControllerState(hand="left", buttons=ControllerButtons(primary=False)),
                "right": ControllerState(hand="right", buttons=ControllerButtons(secondary=False)),
            }
        )

        self.assertTrue(first.save_episode)
        self.assertTrue(first.reset_scene)
        self.assertTrue(first.start_episode)
        self.assertFalse(second.save_episode)
        self.assertFalse(second.reset_scene)
        self.assertFalse(second.start_episode)
        self.assertFalse(released.save_episode)

    def test_v21_recording_prefers_dedicated_worker_python(self):
        config = RecordingConfig.from_mapping({"dataset_format": "v2.1"}, project_root=PROJECT_ROOT)
        recorder = LeRobotEpisodeRecorder(config=config, schema=None, robot_type="openarm", project_root=PROJECT_ROOT)

        self.assertTrue(recorder.worker_process.python.endswith(".venv-lerobot-v21/bin/python"))

    def test_openarm_recording_schema_uses_named_groups_and_cameras(self):
        runtime = load_runtime_config(project_root=PROJECT_ROOT, robot="openarm")
        adapter = OpenArmAdapter.from_mapping(runtime.robot, project_root=PROJECT_ROOT)
        adapter.dof_names = (
            list(runtime.robot["left_arm"]["joints"])
            + list(runtime.robot["grippers"]["left_joints"])
            + list(runtime.robot["right_arm"]["joints"])
            + list(runtime.robot["grippers"]["right_joints"])
        )
        recording = RecordingConfig.from_mapping(runtime.main.get("recording", {}), project_root=PROJECT_ROOT)

        schema = build_recording_schema(adapter, runtime.robot, recording)

        self.assertEqual(schema.state_spec.shape, (16,))
        self.assertEqual(schema.action_spec.shape, (16,))
        self.assertEqual(schema.state_spec.names[0], "openarm_left_joint1")
        self.assertEqual(schema.state_spec.names[7], "left_gripper")
        self.assertEqual(schema.state_spec.names[-1], "right_gripper")
        self.assertEqual(len(schema.camera_specs), 3)
        self.assertEqual(schema.camera_specs[0].shape, (3, 224, 224))

    def test_openarm_recording_vector_normalizes_grippers(self):
        runtime = load_runtime_config(project_root=PROJECT_ROOT, robot="openarm")
        adapter = OpenArmAdapter.from_mapping(runtime.robot, project_root=PROJECT_ROOT)
        adapter.dof_names = (
            list(runtime.robot["left_arm"]["joints"])
            + list(runtime.robot["grippers"]["left_joints"])
            + list(runtime.robot["right_arm"]["joints"])
            + list(runtime.robot["grippers"]["right_joints"])
        )
        adapter.left_gripper_indices = [7, 8]
        adapter.right_gripper_indices = [16, 17]

        current = np.array(
            [
                0.0,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                6.0,
                0.132,
                0.132,
                10.0,
                11.0,
                12.0,
                13.0,
                14.0,
                15.0,
                16.0,
                -1.0,
                -1.0,
            ],
            dtype=float,
        )
        action = RobotAction(joint_positions=current + 0.5)
        recording = RecordingConfig.from_mapping(runtime.main.get("recording", {}), project_root=PROJECT_ROOT)

        names, vector = adapter.get_recording_vector(
            vector_config=recording.state,
            current_joint_positions=current,
            commanded_action=action,
        )

        self.assertEqual(len(names), 16)
        self.assertEqual(vector.shape, (16,))
        self.assertAlmostEqual(vector[7], 0.0)
        self.assertAlmostEqual(vector[-1], 1.0)

    def test_incomplete_lerobot_dataset_root_gets_clear_error(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            dataset_root = Path(temp_dir) / "local" / "quest3-openarm"
            (dataset_root / "meta").mkdir(parents=True)
            (dataset_root / "meta" / "info.json").write_text("{}", encoding="utf-8")

            with self.assertRaisesRegex(RuntimeError, "already exists but is incomplete"):
                _validate_existing_dataset_root(dataset_root)

    def test_v21_lerobot_dataset_root_validation_accepts_episode_layout(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            dataset_root = Path(temp_dir) / "local" / "quest3-openarm"
            meta_dir = dataset_root / "meta"
            meta_dir.mkdir(parents=True)
            (meta_dir / "info.json").write_text('{"codebase_version": "v2.1"}', encoding="utf-8")
            (meta_dir / "episodes.jsonl").write_text("", encoding="utf-8")
            (meta_dir / "episodes_stats.jsonl").write_text("", encoding="utf-8")
            (meta_dir / "tasks.jsonl").write_text("", encoding="utf-8")
            (dataset_root / "data").mkdir()

            _validate_existing_dataset_root(dataset_root, expected_format="v2.1")

    def test_zero_episode_v21_dataset_root_can_be_recreated(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            dataset_root = Path(temp_dir) / "local" / "quest3-openarm"
            meta_dir = dataset_root / "meta"
            meta_dir.mkdir(parents=True)
            (dataset_root / "data").mkdir()
            (dataset_root / "videos").mkdir()
            (meta_dir / "info.json").write_text(
                '{"codebase_version": "v2.1", "total_episodes": 0, "total_frames": 0}',
                encoding="utf-8",
            )
            (meta_dir / "episodes.jsonl").write_text("", encoding="utf-8")

            self.assertTrue(_is_recreatable_empty_dataset_root(dataset_root, "v2.1"))

    def test_lerobot_dataset_root_validation_rejects_format_mismatch(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            dataset_root = Path(temp_dir) / "local" / "quest3-openarm"
            meta_dir = dataset_root / "meta"
            meta_dir.mkdir(parents=True)
            (meta_dir / "info.json").write_text('{"codebase_version": "v3.0"}', encoding="utf-8")

            with self.assertRaisesRegex(RuntimeError, "already exists with format v3.0"):
                _validate_existing_dataset_root(dataset_root, expected_format="v2.1")

    def test_supported_kwargs_filters_version_specific_lerobot_args(self):
        def old_create(repo_id, root, fps, features, robot_type=None, use_videos=True):
            return None

        kwargs = _supported_kwargs(
            old_create,
            {
                "repo_id": "local/test",
                "root": "/tmp/test",
                "fps": 30,
                "features": {},
                "robot_type": "openarm",
                "use_videos": True,
                "streaming_encoding": True,
                "vcodec": "auto",
            },
        )

        self.assertEqual(
            set(kwargs),
            {"repo_id", "root", "fps", "features", "robot_type", "use_videos"},
        )


if __name__ == "__main__":
    unittest.main()
