from __future__ import annotations

import argparse
from copy import deepcopy
import os

from src.teleop_core import TeleopSessionConfig


def merge_mapping(base: dict, override: dict | None) -> dict:
    merged = deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_mapping(merged[key], value)
        else:
            merged[key] = deepcopy(value)
    return merged


def enable_webrtc_streaming(isaac_config: dict) -> None:
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


def recording_cli_requested(args: argparse.Namespace) -> bool:
    return bool(
        args.record
        or args.dataset_root
        or args.dataset_repo_id
        or args.task
        or args.recording_fps is not None
        or args.max_episodes is not None
    )


def build_teleop_arg_parser(robot_name: str, description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--config", dest="config_path", help="Path to the main YAML config file")
    parser.add_argument(
        "--robot",
        default=robot_name,
        help=f"Robot config to load; launcher expects '{robot_name}'",
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


def resolve_common_runtime_settings(
    runtime, args: argparse.Namespace, robot_name: str, robot_display_name: str
) -> tuple[dict, dict, bool, dict]:
    if args.robot != robot_name:
        raise ValueError(f"{robot_display_name} launcher only supports --robot {robot_name}, got '{args.robot}'")

    isaac_config = dict(runtime.main.get("isaac", {}))
    if getattr(args, "webrtc", False):
        enable_webrtc_streaming(isaac_config)
    if args.headless:
        isaac_config["headless"] = True
        simulation = dict(isaac_config.get("simulation", {}))
        simulation["headless"] = True
        isaac_config["simulation"] = simulation

    camera_config = dict(runtime.main.get("cameras", {}))
    if args.disable_cameras:
        camera_config["enabled"] = False

    teleop_config = merge_mapping(
        dict(runtime.main.get("teleop", {})),
        dict(runtime.robot.get("teleop", {})),
    )
    debug_ik = bool(args.debug_ik or teleop_config.get("debug_ik", False))

    recording_config = dict(runtime.main.get("recording", {}))
    
    # Apply standard repo_id and task overrides based on robot name if currently pointing to openarm default
    if recording_config.get("repo_id") == "local/quest3-openarm" and robot_name != "openarm":
        repo_suffix = robot_name.replace("_", "-")
        recording_config["repo_id"] = f"local/quest3-{repo_suffix}"
    if recording_config.get("task") == "Teleoperate OpenArm to complete the task" and robot_display_name != "OpenArm":
        recording_config["task"] = f"Teleoperate {robot_display_name} to complete the task"
        
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
    elif recording_cli_requested(args):
        recording_config["verbose"] = True

    if args.disable_cameras:
        recording_cameras = dict(recording_config.get("cameras", {}))
        recording_cameras["enabled"] = False
        recording_config["cameras"] = recording_cameras

    return isaac_config, camera_config, debug_ik, recording_config


def _optional_tau(value) -> float | None:
    if value is None or value == "":
        return None
    return float(value)


def build_common_teleop_config(
    adapter,
    settings: dict,
    transport_settings: dict | None = None,
    default_workspace_center: list[float] | None = None,
) -> TeleopSessionConfig:
    smoothing = settings.get("smoothing", {})
    transport_settings = transport_settings or {}
    
    position_tau_s = _optional_tau(smoothing.get("position_tau_s"))
    orientation_tau_s = _optional_tau(smoothing.get("orientation_tau_s"))
    
    return TeleopSessionConfig(
        pos_scale=settings.get("position_scale", [1.0, 1.0, 1.0]),
        robot_workspace_center=settings.get("workspace_center", default_workspace_center or [0.3, 0.0, 0.3]),
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
        position_alpha=smoothing.get("position_alpha", 0.9) if position_tau_s is None else None,
        orientation_alpha=smoothing.get("orientation_alpha", 0.9) if orientation_tau_s is None else None,
        position_tau_s=position_tau_s,
        orientation_tau_s=orientation_tau_s,
        gripper_threshold=adapter.gripper_threshold,
        calibration_samples=settings.get("calibration_samples", 30),
        deadman_timeout_s=settings.get("deadman_timeout_ms", 500) / 1000.0,
        hard_timeout_s=settings.get("hard_timeout_ms", 1000) / 1000.0,
        max_target_jump_m=settings.get("max_target_jump_m"),
        max_target_velocity_mps=settings.get("max_target_velocity_mps", 0.4),
        enable_prediction=bool(transport_settings.get("enable_prediction", False)),
        prediction_horizon_s=transport_settings.get("prediction_horizon_ms", 50) / 1000.0,
        jitter_buffer_frames=transport_settings.get("jitter_buffer_frames", 0),
        stale_recovery_alpha=settings.get("stale_recovery_alpha", 0.5),
    )
