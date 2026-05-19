from __future__ import annotations

import argparse
from copy import deepcopy

from src.config_loader import default_project_root, load_runtime_config
from src.launch.bimanual_runtime import run_bimanual_runtime
from src.launch.acone_teleop import _enable_webrtc_streaming
from src.launch.openarm_teleop import build_runtime_config
from src.robot_adapters import FFWBG2Adapter


PROJECT_ROOT = default_project_root()


def _merge_mapping(base: dict, override: dict | None) -> dict:
    merged = deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge_mapping(merged[key], value)
        else:
            merged[key] = deepcopy(value)
    return merged


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Launch FFW BG2 Quest teleoperation")
    parser.add_argument("--config", dest="config_path", help="Path to the main YAML config file")
    parser.add_argument(
        "--robot",
        default="ffw_bg2",
        help="Robot config to load; FFW BG2 launcher expects 'ffw_bg2'",
    )
    parser.add_argument("--headless", action="store_true", help="Force Isaac Sim to run headless")
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
    parser.add_argument("--debug-ik", action="store_true", help="Enable periodic IK target logging")
    parser.add_argument("--record", action="store_true", help="Enable LeRobot dataset recording")
    parser.add_argument("--dataset-root", help="Override recording.root")
    parser.add_argument("--dataset-repo-id", help="Override recording.repo_id")
    parser.add_argument("--task", help="Override recording.task")
    parser.add_argument("--recording-fps", type=int, help="Override recording.fps")
    parser.add_argument("--max-episodes", type=int, help="Stop after this many saved episodes")
    parser.add_argument(
        "--recording-verbose",
        action="store_true",
        help="Print recording progress while recording",
    )
    return parser


def resolve_runtime_settings(args: argparse.Namespace) -> tuple[object, dict, dict, bool, dict]:
    if args.robot != "ffw_bg2":
        raise ValueError(f"FFW BG2 launcher only supports --robot ffw_bg2, got '{args.robot}'")

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

    teleop_config = dict(runtime.main.get("teleop", {}))
    teleop_config.update(dict(runtime.robot.get("teleop", {})))
    debug_ik = bool(args.debug_ik or teleop_config.get("debug_ik", False))
    recording_config = _recording_config(runtime, args)
    if args.disable_cameras:
        recording_cameras = dict(recording_config.get("cameras", {}))
        recording_cameras["enabled"] = False
        recording_config["cameras"] = recording_cameras
    return runtime, isaac_config, camera_config, debug_ik, recording_config


def _recording_config(runtime, args: argparse.Namespace) -> dict:
    recording_config = dict(runtime.main.get("recording", {}))
    if recording_config.get("repo_id") == "local/quest3-openarm":
        recording_config["repo_id"] = "local/quest3-ffw-bg2"
    if recording_config.get("task") == "Teleoperate OpenArm to complete the task":
        recording_config["task"] = "Teleoperate FFW BG2 to complete the task"
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
    if args.recording_verbose or _recording_cli_requested(args):
        recording_config["verbose"] = True
    return recording_config


def _recording_cli_requested(args: argparse.Namespace) -> bool:
    return bool(
        args.record
        or args.dataset_root
        or args.dataset_repo_id
        or args.task
        or args.recording_fps is not None
        or args.max_episodes is not None
    )


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    runtime, isaac_config, camera_config, debug_ik, recording_config = resolve_runtime_settings(args)

    adapter = FFWBG2Adapter.from_mapping(runtime.robot, project_root=PROJECT_ROOT)
    runtime_config = build_runtime_config(
        adapter,
        _merge_mapping(dict(runtime.main.get("teleop", {})), dict(runtime.robot.get("teleop", {}))),
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
