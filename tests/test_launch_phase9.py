import os
import sys
import unittest


TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.join(TESTS_DIR, "..")
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.launch.openarm_teleop import build_arg_parser as build_openarm_parser  # noqa: E402
from src.launch.openarm_teleop import resolve_runtime_settings as resolve_openarm_settings  # noqa: E402
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
        self.assertFalse(recording_config["cameras"]["enabled"])

    def test_openarm_launcher_rejects_non_openarm_robot(self):
        parser = build_openarm_parser()
        args = parser.parse_args(["--robot", "panda"])

        with self.assertRaises(ValueError):
            resolve_openarm_settings(args)

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
