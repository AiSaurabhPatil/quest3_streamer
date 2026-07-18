from __future__ import annotations

import argparse

from src.config_loader import default_project_root, load_runtime_config
from src.launch.bimanual_runtime import run_bimanual_runtime
from src.launch.teleop_utils import (
    build_common_teleop_config,
    build_teleop_arg_parser,
    merge_mapping,
    resolve_common_runtime_settings,
)
from src.robot_adapters import AconeAdapter


PROJECT_ROOT = default_project_root()


def build_arg_parser() -> argparse.ArgumentParser:
    return build_teleop_arg_parser(
        robot_name="acone",
        description="Launch AC One Quest teleoperation",
    )


def resolve_runtime_settings(args: argparse.Namespace) -> tuple[object, dict, dict, bool, dict]:
    runtime = load_runtime_config(
        config_path=args.config_path,
        robot=args.robot,
        project_root=PROJECT_ROOT,
    )
    isaac_config, camera_config, debug_ik, recording_config = resolve_common_runtime_settings(
        runtime=runtime,
        args=args,
        robot_name="acone",
        robot_display_name="AC One",
    )
    return runtime, isaac_config, camera_config, debug_ik, recording_config


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    runtime, isaac_config, camera_config, debug_ik, recording_config = resolve_runtime_settings(args)

    adapter = AconeAdapter.from_mapping(runtime.robot, project_root=PROJECT_ROOT)
    
    merged_teleop_settings = merge_mapping(
        dict(runtime.main.get("teleop", {})),
        dict(runtime.robot.get("teleop", {})),
    )
    
    runtime_config = build_common_teleop_config(
        adapter=adapter,
        settings=merged_teleop_settings,
        transport_settings=runtime.main.get("transport", {}),
        default_workspace_center=[0.35, 0.0, 0.30],
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
