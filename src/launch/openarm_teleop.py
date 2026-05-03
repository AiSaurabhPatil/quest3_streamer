from __future__ import annotations

import argparse

from src.config_loader import default_project_root, load_runtime_config
from src.launch.openarm_runtime import run_openarm_runtime
from src.robot_adapters import OpenArmAdapter
from src.teleop_core import TeleopSessionConfig


PROJECT_ROOT = default_project_root()


def build_runtime_config(
    adapter: OpenArmAdapter,
    settings: dict,
    transport_settings: dict | None = None,
) -> TeleopSessionConfig:
    smoothing = settings.get("smoothing", {})
    transport_settings = transport_settings or {}
    return TeleopSessionConfig(
        pos_scale=settings.get("position_scale", [1.0, 1.0, 1.0]),
        robot_workspace_center=settings.get("workspace_center", [0.3, 0.0, 0.3]),
        left_arm_offset=adapter.left_workspace_offset.tolist(),
        right_arm_offset=adapter.right_workspace_offset.tolist(),
        position_alpha=smoothing.get("position_alpha", 0.9),
        orientation_alpha=smoothing.get("orientation_alpha", 0.9),
        gripper_threshold=adapter.gripper_threshold,
        calibration_samples=settings.get("calibration_samples", 30),
        deadman_timeout_s=settings.get("deadman_timeout_ms", 250) / 1000.0,
        hard_timeout_s=settings.get("hard_timeout_ms", 1000) / 1000.0,
        max_target_jump_m=settings.get("max_target_jump_m"),
        enable_prediction=bool(transport_settings.get("enable_prediction", False)),
        prediction_horizon_s=transport_settings.get("prediction_horizon_ms", 50) / 1000.0,
        jitter_buffer_frames=transport_settings.get("jitter_buffer_frames", 0),
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Launch OpenArm Quest teleoperation")
    parser.add_argument("--config", dest="config_path", help="Path to the main YAML config file")
    parser.add_argument(
        "--robot",
        default="openarm",
        help="Robot config to load; OpenArm launcher expects 'openarm'",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Force Isaac Sim to run headless",
    )
    parser.add_argument(
        "--disable-cameras",
        action="store_true",
        help="Disable camera capture and publishing",
    )
    parser.add_argument(
        "--debug-ik",
        action="store_true",
        help="Enable periodic IK target logging",
    )
    return parser


def resolve_runtime_settings(args: argparse.Namespace) -> tuple[object, dict, dict, bool]:
    if args.robot != "openarm":
        raise ValueError(f"OpenArm launcher only supports --robot openarm, got '{args.robot}'")

    runtime = load_runtime_config(
        config_path=args.config_path,
        robot=args.robot,
        project_root=PROJECT_ROOT,
    )
    isaac_config = dict(runtime.main.get("isaac", {}))
    if args.headless:
        isaac_config["headless"] = True
        simulation = dict(isaac_config.get("simulation", {}))
        simulation["headless"] = True
        isaac_config["simulation"] = simulation

    camera_config = dict(runtime.main.get("cameras", {}))
    if args.disable_cameras:
        camera_config["enabled"] = False

    teleop_config = dict(runtime.main.get("teleop", {}))
    debug_ik = bool(args.debug_ik or teleop_config.get("debug_ik", False))
    return runtime, isaac_config, camera_config, debug_ik


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    runtime, isaac_config, camera_config, debug_ik = resolve_runtime_settings(args)

    adapter = OpenArmAdapter.from_mapping(runtime.robot, project_root=PROJECT_ROOT)
    runtime_config = build_runtime_config(
        adapter,
        runtime.main.get("teleop", {}),
        runtime.main.get("transport", {}),
    )
    return run_openarm_runtime(
        adapter=adapter,
        runtime_config=runtime_config,
        isaac_config=isaac_config,
        camera_config=camera_config,
        debug_ik=debug_ik,
    )


if __name__ == "__main__":
    raise SystemExit(main())
