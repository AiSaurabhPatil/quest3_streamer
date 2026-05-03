import os
import sys
import tempfile
import unittest


TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.join(TESTS_DIR, "..")
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from config_loader import ConfigError, get_dotted_value, load_runtime_config  # noqa: E402


class ConfigLoaderTests(unittest.TestCase):
    def test_load_runtime_config_resolves_main_and_robot_paths(self):
        runtime = load_runtime_config(project_root=PROJECT_ROOT, robot="openarm")

        self.assertEqual(runtime.robot_name, "openarm")
        self.assertTrue(os.path.isabs(runtime.main["paths"]["isaac_sim"]))
        self.assertTrue(os.path.isabs(runtime.main["paths"]["certs"]["cert"]))
        self.assertTrue(os.path.isabs(runtime.robot["usd"]))
        self.assertTrue(runtime.robot["usd"].endswith("openarm_bimanual.usd"))

    def test_loader_supports_active_robot_selection_from_main_config(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            config_dir = os.path.join(tmpdir, "config")
            robots_dir = os.path.join(config_dir, "robots")
            os.makedirs(robots_dir)

            with open(os.path.join(config_dir, "config.yaml"), "w", encoding="utf-8") as handle:
                handle.write(
                    "active_robot: panda\n"
                    "paths:\n"
                    "  isaac_sim: isaac\n"
                    "  certs:\n"
                    "    cert: certs/cert.pem\n"
                    "    key: certs/key.pem\n"
                )

            with open(os.path.join(robots_dir, "panda.yaml"), "w", encoding="utf-8") as handle:
                handle.write(
                    "robot_type: panda\n"
                    "usd: environment.usd\n"
                    "arm:\n"
                    "  frame_name: panda_hand\n"
                    "  joints: [j1, j2]\n"
                    "grippers:\n"
                    "  joints: [g1, g2]\n"
                    "teleop:\n"
                    "  robot_home: [0.5, 0.0, 0.4]\n"
                    "  workspace:\n"
                    "    x_min: 0.0\n"
                    "    x_max: 1.0\n"
                    "    y_min: -1.0\n"
                    "    y_max: 1.0\n"
                    "    z_min: 0.0\n"
                    "    z_max: 1.0\n"
                )

            runtime = load_runtime_config(
                config_path=os.path.join(config_dir, "config.yaml"),
                project_root=tmpdir,
            )

        self.assertEqual(runtime.robot_name, "panda")
        self.assertTrue(runtime.robot["usd"].endswith("environment.usd"))

    def test_loader_raises_clear_error_for_missing_robot_file(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            config_dir = os.path.join(tmpdir, "config")
            os.makedirs(config_dir)

            with open(os.path.join(config_dir, "config.yaml"), "w", encoding="utf-8") as handle:
                handle.write(
                    "active_robot: does_not_exist\n"
                    "paths:\n"
                    "  isaac_sim: isaac\n"
                    "  certs:\n"
                    "    cert: certs/cert.pem\n"
                    "    key: certs/key.pem\n"
                )

            with self.assertRaises(ConfigError) as ctx:
                load_runtime_config(
                    config_path=os.path.join(config_dir, "config.yaml"),
                    project_root=tmpdir,
                )

        self.assertIn("Robot config for 'does_not_exist' was not found", str(ctx.exception))

    def test_get_dotted_value_raises_for_missing_key(self):
        with self.assertRaises(ConfigError):
            get_dotted_value({"server": {}}, "server.websocket_port")


if __name__ == "__main__":
    unittest.main()
