from __future__ import annotations

import argparse
from copy import deepcopy
import os

from src.config_loader import default_project_root, load_runtime_config
from src.launch.bimanual_runtime import run_bimanual_runtime
from src.robot_adapters import AconeAdapter
from src.teleop_core import TeleopSessionConfig


PROJECT_ROOT = default_project_root()


def _merge_mapping(base: dict, override: dict | None) -> dict:
    merged = deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge_mapping(merged[key], value)
        else:
            merged[key] = deepcopy(value)
    return merged


def build_runtime_config(
    adapter: AconeAdapter,
    main_settings: dict,
    robot_settings: dict | None = None,
    transport_settings: dict | None = None,
) -> TeleopSessionConfig:
    settings = _merge_mapping(main_settings, robot_settings or {})
    smoothing = settings.get("smoothing", {})
    transport_settings = transport_settings or {}
    return TeleopSessionConfig(
        pos_scale=settings.get("position_scale", [1.0, 1.0, 1.0]),
        robot_workspace_center=settings.get("workspace_center", [0.35, 0.0, 0.30]),
        left_arm_offset=adapter.left_workspace_offset.tolist(),
        right_arm_offset=adapter.right_workspace_offset.tolist(),
        left_arm_home_orientation=adapter.config["left_arm"].get(
            "home_orientation",
            [1.0, 0.0, 0.0, 0.0],
        ),
        right_arm_home_orientation=adapter.config["right_arm"].get(
            "home_orientation",
            [1.0, 0.0, 0.0, 0.0],
        ),
        position_alpha=smoothing.get("position_alpha", 0.9),
        orientation_alpha=smoothing.get("orientation_alpha", 0.9),
        gripper_threshold=adapter.gripper_threshold,
        calibration_samples=settings.get("calibration_samples", 30),
        deadman_timeout_s=settings.get("deadman_timeout_ms", 500) / 1000.0,
        hard_timeout_s=settings.get("hard_timeout_ms", 1000) / 1000.0,
        max_target_jump_m=settings.get("max_target_jump_m"),
        max_target_velocity_mps=settings.get("max_target_velocity_mps", 0.4),
        enable_prediction=bool(transport_settings.get("enable_prediction", False)),
        prediction_horizon_s=transport_settings.get("prediction_horizon_ms", 50) / 1000.0,
        jitter_buffer_frames=transport_settings.get("jitter_buffer_frames", 0),
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Launch AC One Quest teleoperation")
    parser.add_argument("--config", dest="config_path", help="Path to the main YAML config file")
    parser.add_argument(
        "--robot",
        default="acone",
        help="Robot config to load; AC One launcher expects 'acone'",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Force Isaac Sim to run headless",
    )
    webrtc_group = parser.add_mutually_exclusive_group()
    webrtc_group.add_argument(
        "--webrtc",
        action="store_true",
        help="Run Isaac Sim in WebRTC streaming mode",
    )
    webrtc_group.add_argument(
        "--no-webrtc",
        action="store_true",
        help="Disable WebRTC streaming mode",
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
    parser.add_argument(
        "--record",
        action="store_true",
        help="Enable LeRobot dataset recording",
    )
    parser.add_argument(
        "--dataset-root",
        help="Override recording.root",
    )
    parser.add_argument(
        "--dataset-repo-id",
        help="Override recording.repo_id",
    )
    parser.add_argument(
        "--task",
        help="Override recording.task",
    )
    parser.add_argument(
        "--recording-fps",
        type=int,
        help="Override recording.fps",
    )
    parser.add_argument(
        "--max-episodes",
        type=int,
        help="Stop after this many saved episodes",
    )
    parser.add_argument(
        "--recording-verbose",
        action="store_true",
        help="Print recording progress while recording",
    )
    return parser


def resolve_runtime_settings(args: argparse.Namespace) -> tuple[object, dict, dict, bool, dict]:
    if args.robot != "acone":
        raise ValueError(f"AC One launcher only supports --robot acone, got '{args.robot}'")

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
    if args.webrtc:
        _enable_webrtc_streaming(isaac_config)

    camera_config = dict(runtime.main.get("cameras", {}))
    if args.disable_cameras:
        camera_config["enabled"] = False

    teleop_config = _merge_mapping(
        dict(runtime.main.get("teleop", {})),
        dict(runtime.robot.get("teleop", {})),
    )
    debug_ik = bool(args.debug_ik or teleop_config.get("debug_ik", False))
    recording_config = dict(runtime.main.get("recording", {}))
    if recording_config.get("repo_id") == "local/quest3-openarm":
        recording_config["repo_id"] = "local/quest3-acone"
    if recording_config.get("task") == "Teleoperate OpenArm to complete the task":
        recording_config["task"] = "Teleoperate AC One to complete the task"
    recording_cameras = dict(recording_config.get("cameras", {}))
    if tuple(recording_cameras.get("include", ())) == ("head", "wrist_left", "wrist_right"):
        recording_cameras["include"] = list(runtime.robot.get("cameras", {}).keys())
        recording_config["cameras"] = recording_cameras
    if args.record:
        recording_config["enabled"] = True
    if args.dataset_root:
        recording_config["root"] = args.dataset_root
        recording_config["enabled"] = True
    if args.dataset_repo_id:
        recording_config["repo_id"] = args.dataset_repo_id
        recording_config["enabled"] = True
    if args.task:
        recording_config["task"] = args.task
        recording_config["enabled"] = True
    if args.recording_fps is not None:
        recording_config["fps"] = args.recording_fps
        recording_config["enabled"] = True
    if args.max_episodes is not None:
        recording_config["max_episodes"] = args.max_episodes
        recording_config["enabled"] = True
    if args.recording_verbose:
        recording_config["verbose"] = True
    elif _recording_cli_requested(args):
        recording_config["verbose"] = True
    if args.disable_cameras:
        recording_cameras = dict(recording_config.get("cameras", {}))
        recording_cameras["enabled"] = False
        recording_config["cameras"] = recording_cameras
    return runtime, isaac_config, camera_config, debug_ik, recording_config


def _recording_cli_requested(args: argparse.Namespace) -> bool:
    return bool(
        args.record
        or args.dataset_root
        or args.dataset_repo_id
        or args.task
        or args.recording_fps is not None
        or args.max_episodes is not None
    )


def _enable_webrtc_streaming(isaac_config: dict) -> None:
    isaac_sim_path = os.environ.get("ISAAC_SIM_PATH", "/home/saurabh/isaac_sim")
    isaac_config["experience"] = os.path.join(isaac_sim_path, "apps", "isaacsim.exp.full.streaming.kit")
    isaac_config["webrtc_streaming"] = True
    isaac_config["headless"] = True
    simulation = dict(isaac_config.get("simulation", {}))
    simulation["headless"] = True
    simulation["hide_ui"] = False
    webrtc_gpu = int(os.environ.get("ISAAC_WEBRTC_GPU", "0"))
    simulation["active_gpu"] = webrtc_gpu
    simulation["physics_gpu"] = webrtc_gpu
    simulation["multi_gpu"] = False
    simulation["max_gpu_count"] = 1
    extra_args = list(simulation.get("extra_args", []))
    for arg in (
        "--/app/window/drawMouse=true",
        "--/app/livestream/quitOnSessionEnded=false",
        "--/renderer/multiGpu/enabled=false",
        "--/renderer/multiGpu/maxGpuCount=1",
    ):
        if arg not in extra_args:
            extra_args.append(arg)
    simulation["extra_args"] = extra_args
    isaac_config["simulation"] = simulation


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    runtime, isaac_config, camera_config, debug_ik, recording_config = resolve_runtime_settings(args)

    adapter = AconeAdapter.from_mapping(runtime.robot, project_root=PROJECT_ROOT)
    runtime_config = build_runtime_config(
        adapter,
        dict(runtime.main.get("teleop", {})),
        dict(runtime.robot.get("teleop", {})),
        runtime.main.get("transport", {}),
    )
    return run_bimanual_runtime(
        adapter=adapter,
        runtime_config=runtime_config,
        isaac_config=isaac_config,
        camera_config=camera_config,
        debug_ik=debug_ik,
        recording_config=recording_config,
        project_root=PROJECT_ROOT,
    )


if __name__ == "__main__":
    raise SystemExit(main())
