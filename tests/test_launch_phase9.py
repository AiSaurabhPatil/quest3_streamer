import os
import sys
import unittest


TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.join(TESTS_DIR, "..")
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.launch.openarm_teleop import build_arg_parser as build_openarm_parser  # noqa: E402
from src.launch.openarm_teleop import resolve_runtime_settings as resolve_openarm_settings  # noqa: E402
from src.launch.acone_teleop import build_arg_parser as build_acone_parser  # noqa: E402
from src.launch.acone_teleop import resolve_runtime_settings as resolve_acone_settings  # noqa: E402
from src.launch.ffw_bg2_teleop import build_arg_parser as build_ffw_bg2_parser  # noqa: E402
from src.launch.ffw_bg2_teleop import resolve_runtime_settings as resolve_ffw_bg2_settings  # noqa: E402
from src.launch.panda_teleop import build_arg_parser as build_panda_parser  # noqa: E402
from src.launch.panda_teleop import resolve_runtime_settings as resolve_panda_settings  # noqa: E402
from src.launch.webxr_bridge import build_arg_parser as build_bridge_parser  # noqa: E402


class LaunchPhase9Tests(unittest.TestCase):
    def test_openarm_cli_overrides_headless_disable_cameras_and_debug_ik(self):
        parser = build_openarm_parser()
        args = parser.parse_args(
            [
                "--headless",
                "--disable-cameras",
                "--debug-ik",
                "--record",
                "--dataset-root",
                "tmp-datasets",
                "--dataset-repo-id",
                "local/test-openarm",
                "--task",
                "Test task",
                "--recording-fps",
                "15",
                "--max-episodes",
                "5",
            ]
        )

        runtime, isaac_config, camera_config, debug_ik, recording_config = resolve_openarm_settings(args)

        self.assertEqual(runtime.robot_name, "openarm")
        self.assertTrue(isaac_config["headless"])
        self.assertTrue(isaac_config["simulation"]["headless"])
        self.assertFalse(camera_config["enabled"])
        self.assertTrue(debug_ik)
        self.assertTrue(recording_config["enabled"])
        self.assertEqual(recording_config["root"], "tmp-datasets")
        self.assertEqual(recording_config["repo_id"], "local/test-openarm")
        self.assertEqual(recording_config["task"], "Test task")
        self.assertEqual(recording_config["fps"], 15)
        self.assertEqual(recording_config["max_episodes"], 5)
        self.assertTrue(recording_config["verbose"])
        self.assertFalse(recording_config["cameras"]["enabled"])

    def test_openarm_launcher_rejects_non_openarm_robot(self):
        parser = build_openarm_parser()
        args = parser.parse_args(["--robot", "panda"])

        with self.assertRaises(ValueError):
            resolve_openarm_settings(args)

    def test_acone_recording_uses_acone_camera_names(self):
        parser = build_acone_parser()
        args = parser.parse_args(["--record"])

        runtime, _, _, _, recording_config = resolve_acone_settings(args)

        self.assertEqual(runtime.robot_name, "acone")
        self.assertTrue(recording_config["enabled"])
        self.assertTrue(recording_config["verbose"])
        self.assertEqual(recording_config["repo_id"], "local/quest3-acone")
        self.assertEqual(
            recording_config["cameras"]["include"],
            ["head_camera", "left_wrist_camera", "right_wrist_camera"],
        )

    def test_acone_webrtc_forces_headless_streaming_experience(self):
        parser = build_acone_parser()
        args = parser.parse_args(["--webrtc"])

        _, isaac_config, _, _, _ = resolve_acone_settings(args)

        self.assertTrue(isaac_config["headless"])
        self.assertTrue(isaac_config["simulation"]["headless"])
        self.assertFalse(isaac_config["simulation"]["hide_ui"])
        self.assertFalse(isaac_config["simulation"]["multi_gpu"])
        self.assertEqual(isaac_config["simulation"]["max_gpu_count"], 1)
        self.assertEqual(isaac_config["simulation"]["active_gpu"], 0)
        self.assertEqual(isaac_config["simulation"]["physics_gpu"], 0)
        self.assertTrue(isaac_config["webrtc_streaming"])
        self.assertTrue(isaac_config["experience"].endswith("isaacsim.exp.full.streaming.kit"))

    def test_ffw_bg2_recording_uses_robot_defaults(self):
        parser = build_ffw_bg2_parser()
        args = parser.parse_args(["--record"])

        runtime, _, _, _, recording_config = resolve_ffw_bg2_settings(args)

        self.assertEqual(runtime.robot_name, "ffw_bg2")
        self.assertTrue(recording_config["enabled"])
        self.assertTrue(recording_config["verbose"])
        self.assertEqual(recording_config["repo_id"], "local/quest3-ffw-bg2")
        self.assertEqual(
            recording_config["cameras"]["include"],
            ["head_camera", "left_wrist_camera", "right_wrist_camera"],
        )

    def test_ffw_bg2_launcher_rejects_non_ffw_bg2_robot(self):
        parser = build_ffw_bg2_parser()
        args = parser.parse_args(["--robot", "openarm"])

        with self.assertRaises(ValueError):
            resolve_ffw_bg2_settings(args)

    def test_ffw_bg2_webrtc_forces_headless_streaming_experience(self):
        parser = build_ffw_bg2_parser()
        args = parser.parse_args(["--webrtc"])

        _, isaac_config, _, _, _ = resolve_ffw_bg2_settings(args)

        self.assertTrue(isaac_config["headless"])
        self.assertTrue(isaac_config["simulation"]["headless"])
        self.assertFalse(isaac_config["simulation"]["hide_ui"])
        self.assertFalse(isaac_config["simulation"]["multi_gpu"])
        self.assertEqual(isaac_config["simulation"]["max_gpu_count"], 1)
        self.assertEqual(isaac_config["simulation"]["active_gpu"], 0)
        self.assertEqual(isaac_config["simulation"]["physics_gpu"], 0)
        self.assertTrue(isaac_config["webrtc_streaming"])
        self.assertTrue(isaac_config["experience"].endswith("isaacsim.exp.full.streaming.kit"))

    def test_panda_cli_overrides_headless_and_disable_cameras(self):
        parser = build_panda_parser()
        args = parser.parse_args(["--headless", "--disable-cameras"])

        runtime, isaac_config, camera_config = resolve_panda_settings(args)

        self.assertEqual(runtime.robot_name, "panda")
        self.assertTrue(isaac_config["headless"])
        self.assertTrue(isaac_config["simulation"]["headless"])
        self.assertFalse(camera_config["enabled"])

    def test_bridge_parser_defaults_to_configured_ports_and_certs(self):
        parser = build_bridge_parser([])
        args = parser.parse_args([])

        self.assertEqual(args.port, 9999)
        self.assertTrue(str(args.cert).endswith("certs/cert.pem"))
        self.assertTrue(str(args.key).endswith("certs/key.pem"))


if __name__ == "__main__":
    unittest.main()
