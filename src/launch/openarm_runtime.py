from __future__ import annotations

from src.isaac_backend import CameraImagePublishers, CameraManager, IsaacApp, JointStatePublisher
from src.launch.control_metrics import ControlMetricsReporter
from src.launch.controller_provider import build_controller_provider_class, import_ros_interfaces
from src.robot_adapters import OpenArmAdapter
from src.teleop_core import BimanualTeleopSession, TeleopSessionConfig


def run_openarm_runtime(
    *,
    adapter: OpenArmAdapter,
    runtime_config: TeleopSessionConfig,
    isaac_config: dict,
    camera_config: dict,
    debug_ik: bool,
) -> int:
    teleop_session = BimanualTeleopSession(runtime_config)
    isaac_app = IsaacApp(isaac_config)

    rclpy = None
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
            node_name="isaac_openarm_teleop",
            hands=("left", "right"),
        )

        print(f"[Init] Loading stage from {adapter.usd_path}...")
        isaac_app.load_stage(adapter.usd_path)

        print("[Init] Creating World...")
        world = isaac_app.create_world(stage_units_in_meters=1.0)

        print("[Init] Loading OpenArm robot...")
        try:
            adapter.load(world, world.stage)
        except RuntimeError as exc:
            print(f"[ERROR] {exc}")
            return 1
        print(f"[Init] Found robot at: {adapter.robot_prim_path}")

        print("[Init] Loading IK Solvers...")
        ik_enabled = _initialize_ik(adapter)

        print("[UI] Minimizing panels for Zoom Mode...")
        isaac_app.hide_ui_panels()

        print("[Init] Resetting World...")
        isaac_app.reset_world()

        print("[Init] Getting joint information...")
        adapter.initialize_joint_mappings()
        dof_names = adapter.get_joint_names()
        _print_joint_info(adapter, dof_names)

        print("[Init] Initializing ROS2...")
        rclpy.init()
        ros_started = True
        controller_provider = QuestControllerProvider()
        joint_state_publisher = JointStatePublisher(controller_provider)

        camera_specs = adapter.get_camera_specs()
        camera_publishers = None
        if camera_config.get("enabled", True) and camera_specs:
            camera_publishers = CameraImagePublishers(controller_provider, camera_specs)

        camera_manager = CameraManager(
            stage=world.stage,
            camera_specs=camera_specs,
            camera_publishers=camera_publishers,
            viewport_cameras=adapter.get_viewport_cameras(),
            config=camera_config,
        ).start()

        _print_ready(adapter, runtime_config, camera_manager)
        _run_control_loop(
            isaac_app=isaac_app,
            adapter=adapter,
            teleop_session=teleop_session,
            controller_provider=controller_provider,
            joint_state_publisher=joint_state_publisher,
            camera_manager=camera_manager,
            dof_names=dof_names,
            rclpy=rclpy,
            ik_enabled=ik_enabled,
            debug_ik=debug_ik,
        )

        _print_session_statistics(adapter, camera_manager)
        return 0
    finally:
        if camera_manager is not None:
            camera_manager.close()
        if controller_provider is not None:
            controller_provider.destroy_node()
        if ros_started and rclpy is not None:
            rclpy.shutdown()
        isaac_app.close()


def _initialize_ik(adapter: OpenArmAdapter) -> bool:
    try:
        ik_enabled = adapter.initialize_ik()
        if ik_enabled:
            print("[Init] IK Solvers loaded successfully!")
            return True

        diagnostics = adapter.get_diagnostics()
        print(
            "[WARNING] Could not load IK Solvers: "
            f"{diagnostics.details.get('ik_init_error', 'unknown error')}"
        )
    except Exception as exc:
        print(f"[WARNING] Could not load IK Solvers: {exc}")

    print("[INFO] Falling back to joint position control mode")
    return False


def _print_joint_info(adapter: OpenArmAdapter, dof_names: list[str]) -> None:
    print(f"[Info] Available DOFs: {dof_names}")
    print(f"[Info] Left arm indices: {adapter.left_arm_indices}")
    print(f"[Info] Right arm indices: {adapter.right_arm_indices}")
    print(f"[Info] Left gripper indices: {adapter.left_gripper_indices}")
    print(f"[Info] Right gripper indices: {adapter.right_gripper_indices}")


def _print_ready(
    adapter: OpenArmAdapter,
    runtime_config: TeleopSessionConfig,
    camera_manager: CameraManager,
) -> None:
    print("=" * 60)
    print("OpenArm Bimanual Teleop Ready")
    print(f"Loaded: {adapter.usd_path}")
    print("=" * 60)
    print("Controls:")
    print("  - Left Quest Controller  -> Left Arm")
    print("  - Right Quest Controller -> Right Arm")
    print("  - Trigger/Grip -> Close Gripper")
    print("  - A/X Button -> Camera Switch")
    print(f"  - Deadman Timeout -> {runtime_config.deadman_timeout_s * 1000.0:.0f} ms")
    print("=" * 60)
    print(
        "[Camera] Viewport cameras: "
        f"{camera_manager.viewport_camera_names or ['Perspective']}"
    )


def _run_control_loop(
    *,
    isaac_app: IsaacApp,
    adapter: OpenArmAdapter,
    teleop_session: BimanualTeleopSession,
    controller_provider,
    joint_state_publisher: JointStatePublisher,
    camera_manager: CameraManager,
    dof_names: list[str],
    rclpy,
    ik_enabled: bool,
    debug_ik: bool,
) -> None:
    last_camera_switch = False
    ik_disabled_reported = False
    control_metrics = ControlMetricsReporter()

    while isaac_app.is_running():
        rclpy.spin_once(controller_provider, timeout_sec=0.0)
        session_update = teleop_session.update(controller_provider.latest())

        _log_session_events(session_update.events, controller_provider)
        last_camera_switch = _maybe_switch_camera(
            controller_provider,
            camera_manager,
            last_camera_switch,
        )

        if not session_update.ready:
            _handle_not_ready(
                isaac_app,
                controller_provider,
                session_update,
                teleop_session.config,
            )
            continue

        if adapter.get_current_joint_positions() is None:
            isaac_app.step(render=True)
            continue

        _maybe_print_ik_debug(adapter, session_update, debug_ik)
        action = adapter.compute_action(session_update.targets)

        if not ik_enabled and not ik_disabled_reported:
            print("[INFO] IK disabled; holding arm joints while grippers remain responsive")
            ik_disabled_reported = True

        adapter.apply_action(action)
        teleop_session.mark_isaac_apply()
        stamp = controller_provider.get_clock().now().to_msg()
        joint_state_publisher.publish(dof_names, action.joint_positions, stamp=stamp)
        camera_manager.update(stamp=stamp)
        control_metrics.record(session_update, adapter, controller_provider.get_logger())

        isaac_app.step(render=True)


def _log_session_events(events, controller_provider) -> None:
    for event in events:
        if event.level == "warning":
            controller_provider.get_logger().warn(event.message)
        else:
            controller_provider.get_logger().info(event.message)


def _maybe_switch_camera(controller_provider, camera_manager, last_camera_switch: bool) -> bool:
    camera_switch_pressed = controller_provider.camera_switch_pressed
    if camera_switch_pressed and not last_camera_switch:
        switched_camera = camera_manager.switch_viewport_camera_next()
        if switched_camera is not None:
            camera_name, camera_path = switched_camera
            print(f"[Camera] Switched to: {camera_name} ({camera_path})")
    return camera_switch_pressed


def _handle_not_ready(
    isaac_app: IsaacApp,
    controller_provider,
    session_update,
    runtime_config: TeleopSessionConfig,
) -> None:
    if controller_provider.total_pose_count > 0:
        calibration_status = session_update.calibration_status
        left_status = (
            "CALIBRATED"
            if session_update.left_state.calibrated
            else (
                "Calibrating "
                f"({calibration_status.left_samples}/{runtime_config.calibration_samples})"
            )
        )
        right_status = (
            "CALIBRATED"
            if session_update.right_state.calibrated
            else (
                "Calibrating "
                f"({calibration_status.right_samples}/{runtime_config.calibration_samples})"
            )
        )
        if controller_provider.total_pose_count % 30 == 1:
            print(f"[Calibration] Left: {left_status} | Right: {right_status}")
    else:
        controller_provider.maybe_report_waiting_for_controllers()
    isaac_app.step(render=True)


def _maybe_print_ik_debug(adapter: OpenArmAdapter, session_update, debug_ik: bool) -> None:
    diagnostics = adapter.get_diagnostics()
    frame_count = sum(diagnostics.counters.values())
    if debug_ik and frame_count % 100 == 0:
        print(
            "[IK Debug] Left target: "
            f"pos={session_update.targets.left_ee.position_xyz}, "
            f"rot={session_update.targets.left_ee.orientation_wxyz}"
        )
        print(
            "[IK Debug] Right target: "
            f"pos={session_update.targets.right_ee.position_xyz}, "
            f"rot={session_update.targets.right_ee.orientation_wxyz}"
        )


def _print_session_statistics(adapter: OpenArmAdapter, camera_manager: CameraManager) -> None:
    print("\n" + "=" * 60)
    diagnostics = adapter.get_diagnostics()
    print("Session Statistics:")
    print(
        "  Left Arm  - IK Success: "
        f"{diagnostics.counters.get('left_ik_success', 0)}, "
        f"IK Fail: {diagnostics.counters.get('left_ik_fail', 0)}"
    )
    print(
        "  Right Arm - IK Success: "
        f"{diagnostics.counters.get('right_ik_success', 0)}, "
        f"IK Fail: {diagnostics.counters.get('right_ik_fail', 0)}"
    )
    print(
        "  Camera    - Published: "
        f"{camera_manager.diagnostics.published_frames}, "
        f"Dropped: {camera_manager.diagnostics.dropped_frames}, "
        f"Errors: {camera_manager.diagnostics.errors}"
    )
    print("=" * 60)
