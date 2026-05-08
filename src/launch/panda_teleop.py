from __future__ import annotations

import argparse

from src.config_loader import default_project_root, load_runtime_config
from src.isaac_backend import CameraManager, IsaacApp
from src.launch.controller_provider import build_controller_provider_class, import_ros_interfaces
from src.robot_adapters import PandaAdapter
from src.teleop_core import (
    SingleArmTeleopSession,
    TeleopSessionConfig,
)


PROJECT_ROOT = default_project_root()


def build_runtime_config(
    adapter: PandaAdapter,
    settings: dict,
    transport_settings: dict | None = None,
) -> TeleopSessionConfig:
    smoothing = settings.get("smoothing", {})
    transport_settings = transport_settings or {}
    return TeleopSessionConfig(
        pos_scale=settings.get("position_scale", adapter.pos_scale),
        robot_workspace_center=adapter.robot_home,
        position_alpha=smoothing.get("position_alpha", 0.9),
        orientation_alpha=smoothing.get("orientation_alpha", 0.9),
        gripper_threshold=adapter.gripper_threshold,
        calibration_samples=settings.get("calibration_samples", adapter.calibration_samples),
        deadman_timeout_s=settings.get("deadman_timeout_ms", 500) / 1000.0,
        hard_timeout_s=settings.get("hard_timeout_ms", 1000) / 1000.0,
        max_target_jump_m=settings.get("max_target_jump_m"),
        max_target_velocity_mps=settings.get("max_target_velocity_mps", 0.4),
        workspace_bounds=adapter.workspace_bounds,
        enable_prediction=bool(transport_settings.get("enable_prediction", False)),
        prediction_horizon_s=transport_settings.get("prediction_horizon_ms", 50) / 1000.0,
        jitter_buffer_frames=transport_settings.get("jitter_buffer_frames", 0),
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Launch Panda Quest teleoperation")
    parser.add_argument("--config", dest="config_path", help="Path to the main YAML config file")
    parser.add_argument(
        "--robot",
        default="panda",
        help="Robot config to load; Panda launcher expects 'panda'",
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
        help="Accepted for CLI parity; Panda launcher does not currently emit extra IK logs",
    )
    return parser


def resolve_runtime_settings(args: argparse.Namespace) -> tuple[object, dict, dict]:
    if args.robot != "panda":
        raise ValueError(f"Panda launcher only supports --robot panda, got '{args.robot}'")

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

    return runtime, isaac_config, camera_config


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    runtime, isaac_config, camera_config = resolve_runtime_settings(args)

    adapter = PandaAdapter.from_mapping(runtime.robot, project_root=PROJECT_ROOT)
    runtime_config = build_runtime_config(
        adapter,
        runtime.main.get("teleop", {}),
        runtime.main.get("transport", {}),
    )
    teleop_session = SingleArmTeleopSession(
        runtime_config,
        hand="right",
        home_position=adapter.robot_home,
    )
    isaac_app = IsaacApp(isaac_config)

    controller_provider = None
    camera_manager = None
    ros_started = False

    try:
        print("[Init] Warming up Isaac Sim...")
        isaac_app.start()
        rclpy, Node, PoseStamped, Joy = import_ros_interfaces()
        QuestControllerProvider = build_controller_provider_class(
            Node,
            PoseStamped,
            Joy,
            node_name="isaac_panda_teleop",
            hands=("right",),
        )

        print(f"[Init] Loading stage from {adapter.usd_path}...")
        isaac_app.load_stage(adapter.usd_path)

        print("[Init] Creating World...")
        world = isaac_app.create_world(stage_units_in_meters=1.0)

        print("[Init] Adding Franka...")
        adapter.load(world, world.stage)

        print("[Init] Loading IK Solver...")
        adapter.initialize_ik()

        print("[UI] Minimizing panels for Zoom Mode...")
        isaac_app.hide_ui_panels()

        print("[Init] Resetting World...")
        isaac_app.reset_world()

        adapter.initialize_joint_mappings()

        print("[Init] Initializing ROS2...")
        rclpy.init()
        ros_started = True
        controller_provider = QuestControllerProvider()
        camera_manager = CameraManager(
            stage=world.stage,
            camera_specs=adapter.get_camera_specs(),
            viewport_cameras=adapter.get_viewport_cameras(),
            config=camera_config,
        ).start()

        print("=" * 60)
        print("Isaac Sim Teleop (USD Mode)")
        print(f"Loaded: {adapter.usd_path}")
        print(f"Deadman Timeout: {runtime_config.deadman_timeout_s * 1000.0:.0f} ms")
        print("=" * 60)

        last_button_a = False

        while isaac_app.is_running():
            rclpy.spin_once(controller_provider, timeout_sec=0.0)
            session_update = teleop_session.update(
                controller_provider.latest_hand("right")
            )

            for event in session_update.events:
                if event.level == "warning":
                    controller_provider.get_logger().warn(event.message)
                else:
                    controller_provider.get_logger().info(event.message)

            if controller_provider.camera_switch_pressed and not last_button_a:
                switched_camera = camera_manager.switch_viewport_camera_next() if camera_manager else None
                if switched_camera is not None:
                    _, camera_path = switched_camera
                    print(f"[Camera] Switched to {camera_path}")
            last_button_a = controller_provider.camera_switch_pressed

            if not session_update.ready:
                if controller_provider.total_pose_count > 0:
                    if controller_provider.total_pose_count % 30 == 1:
                        print(
                            "[Calibration] Right: "
                            f"{session_update.calibration_samples}/{runtime_config.calibration_samples}"
                        )
                else:
                    controller_provider.maybe_report_waiting_for_controllers()
                isaac_app.step(render=True)
                continue

            action = adapter.compute_action(session_update.targets)
            adapter.apply_action(action)
            teleop_session.mark_isaac_apply()
            isaac_app.step(render=True)

        diagnostics = adapter.get_diagnostics()
        print("\n" + "=" * 60)
        print("Session Statistics:")
        print(
            "  Arm - IK Success: "
            f"{diagnostics.counters.get('ik_success', 0)}, "
            f"IK Fail: {diagnostics.counters.get('ik_fail', 0)}"
        )
        print("=" * 60)
        return 0
    finally:
        if camera_manager is not None:
            camera_manager.close()
        if controller_provider is not None:
            controller_provider.destroy_node()
        if ros_started:
            rclpy.shutdown()
        isaac_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
