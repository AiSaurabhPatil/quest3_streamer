from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from src.isaac_backend import (
    CameraImagePublishers,
    CameraManager,
    DomainRandomizer,
    FFWBG2DomainRandomizer,
    IsaacApp,
    JointStatePublisher,
)
from src.launch.control_metrics import ControlMetricsReporter
from src.launch.controller_provider import build_controller_provider_class, import_ros_interfaces
from src.recording import (
    ButtonEdgeMapper,
    LeRobotEpisodeRecorder,
    RecordingConfig,
    RecordingFrameSnapshot,
    build_recording_schema,
)
from src.robot_adapters import OpenArmAdapter
from src.teleop_core import BimanualTeleopSession, TeleopSessionConfig


@dataclass
class RecordingLoopState:
    target_episodes: int | None
    recording_active: bool = False
    awaiting_save_completion: bool = False
    awaiting_discard_completion: bool = False
    last_saved_reported: int = 0
    last_discarded_reported: int = 0
    last_progress_report_s: float = 0.0

    def has_reached_target(self, saved_episodes: int) -> bool:
        return self.target_episodes is not None and saved_episodes >= self.target_episodes

    def progress_text(self, saved_episodes: int | None = None) -> str:
        completed = self.last_saved_reported if saved_episodes is None else saved_episodes
        if self.target_episodes is None:
            return f"{completed} saved"
        return f"{completed}/{self.target_episodes} saved"

    def next_episode_label(self, saved_episodes: int) -> str:
        next_episode = saved_episodes + 1
        if self.target_episodes is None:
            return f"episode {next_episode}"
        return f"episode {next_episode}/{self.target_episodes}"


def run_bimanual_runtime(
    *,
    adapter: OpenArmAdapter,
    runtime_config: TeleopSessionConfig,
    isaac_config: dict,
    camera_config: dict,
    debug_ik: bool,
    recording_config: dict | None = None,
    project_root: str | None = None,
) -> int:
    robot_display_name = _adapter_display_name(adapter)
    recording_settings = RecordingConfig.from_mapping(
        recording_config,
        project_root=project_root or adapter.project_root,
    )
    quiet_recording = recording_settings.enabled and not recording_settings.verbose
    if quiet_recording:
        isaac_config = _with_quiet_isaac_logging(isaac_config)
    isaac_app = IsaacApp(isaac_config)

    rclpy = None
    controller_provider = None
    camera_manager = None
    domain_randomizer = None
    recorder = None
    recording_schema = None
    ros_started = False

    try:
        print("[Init] Warming up Isaac Sim...")
        isaac_app.start()
        rclpy, Node, PoseStamped, Joy = import_ros_interfaces()
        QuestControllerProvider = build_controller_provider_class(
            Node,
            PoseStamped,
            Joy,
            node_name=f"isaac_{adapter.config.get('robot_type', 'bimanual')}_teleop",
            hands=("left", "right"),
        )

        print(f"[Init] Loading stage from {adapter.usd_path}...")
        isaac_app.load_stage(adapter.usd_path)

        print("[Init] Creating World...")
        world = isaac_app.create_world(stage_units_in_meters=1.0)

        print(f"[Init] Loading {robot_display_name} robot...")
        try:
            adapter.load(world, world.stage)
        except RuntimeError as exc:
            print(f"[ERROR] {exc}")
            return 1
        print(f"[Init] Found robot at: {adapter.robot_prim_path}")

        randomizer_cls = (
            FFWBG2DomainRandomizer
            if adapter.config.get("robot_type") == "ffw_bg2"
            else DomainRandomizer
        )
        domain_randomizer = randomizer_cls(world.stage, adapter.config.get("domain_randomization"))
        domain_randomizer.initialize()
        print(f"[Init] Domain randomizer: {randomizer_cls.__name__} (enabled={domain_randomizer.enabled})")

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
        if hasattr(adapter, "configure_runtime_home_from_current_pose"):
            runtime_config = adapter.configure_runtime_home_from_current_pose(runtime_config)
        teleop_session = BimanualTeleopSession(runtime_config)

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

        if recording_settings.enabled:
            if not camera_specs:
                recording_cameras = dict(recording_config.get("cameras", {}) if recording_config else {})
                recording_cameras["enabled"] = False
                recording_settings = RecordingConfig.from_mapping(
                    {
                        **(recording_config or {}),
                        "cameras": recording_cameras,
                    },
                    project_root=project_root or adapter.project_root,
                )
            recording_schema = build_recording_schema(adapter, adapter.config, recording_settings)
            try:
                recorder = LeRobotEpisodeRecorder(
                    config=recording_settings,
                    schema=recording_schema,
                    robot_type=adapter.get_recording_robot_type(),
                    project_root=project_root or adapter.project_root,
                ).start()
            except Exception as exc:
                print(f"[Recording] Failed to start LeRobot recorder: {exc}")
                return 1

        _print_ready(adapter, runtime_config, camera_manager, recording_settings)
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
            recording_settings=recording_settings,
            domain_randomizer=domain_randomizer,
            recorder=recorder,
            recording_schema=recording_schema,
        )

        _print_session_statistics(adapter, camera_manager, recorder)
        return 0
    finally:
        if recorder is not None:
            _cleanup_resource("close recorder", recorder.close)
        if camera_manager is not None:
            _cleanup_resource("close camera manager", camera_manager.close)
        if controller_provider is not None:
            _cleanup_resource("destroy ROS controller node", controller_provider.destroy_node)
        if ros_started and rclpy is not None:
            _cleanup_resource("shutdown ROS", rclpy.shutdown)
        _cleanup_resource("close Isaac app", isaac_app.close)


def _cleanup_resource(label: str, cleanup) -> None:
    try:
        cleanup()
    except Exception as exc:
        if label == "destroy ROS controller node" and _is_known_ros_destroy_node_cleanup_error(exc):
            return
        print(f"[Cleanup] Warning: failed to {label}: {type(exc).__name__}: {exc}")


def _is_known_ros_destroy_node_cleanup_error(exc: Exception) -> bool:
    return isinstance(exc, ValueError) and str(exc) == "list.remove(x): x not in list"


def _with_quiet_isaac_logging(isaac_config: dict) -> dict:
    quiet_config = dict(isaac_config)
    quiet_config["quiet_logging"] = True
    simulation = dict(quiet_config.get("simulation", {}))
    extra_args = list(simulation.get("extra_args", []))
    for arg in (
        "--/log/level=error",
        "--/app/enableDeveloperWarnings=false",
    ):
        if arg not in extra_args:
            extra_args.append(arg)
    simulation["extra_args"] = extra_args
    quiet_config["simulation"] = simulation
    return quiet_config


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
    recording_config: RecordingConfig,
) -> None:
    robot_display_name = _adapter_display_name(adapter)
    print("=" * 60)
    print(f"{robot_display_name} Bimanual Teleop Ready")
    print(f"Loaded: {adapter.usd_path}")
    print("=" * 60)
    print("Controls:")
    print("  - Left Quest Controller  -> Left Arm")
    print("  - Right Quest Controller -> Right Arm")
    print("  - Trigger/Grip -> Close Gripper")
    print("  - A -> Camera Switch")
    print("  - B -> Scene Reset")
    print("  - X -> Save Episode")
    print("  - Y -> Start Episode Recording")
    print(f"  - Deadman Timeout -> {runtime_config.deadman_timeout_s * 1000.0:.0f} ms")
    print("=" * 60)
    print(
        "[Camera] Viewport cameras: "
        f"{camera_manager.viewport_camera_names or ['Perspective']}"
    )
    if recording_config.enabled and recording_config.verbose:
        print(
            "[Recording] Enabled: "
            f"repo_id={recording_config.repo_id}, root={recording_config.root}, "
            f"fps={recording_config.fps}, max_episodes={recording_config.max_episodes}"
        )
        print("[Recording] Press Y to start each episode after your setup is ready")
    elif not recording_config.enabled:
        print("[Recording] Disabled")


def _adapter_display_name(adapter: OpenArmAdapter) -> str:
    return str(
        adapter.config.get("display_name")
        or adapter.config.get("robot_type")
        or adapter.__class__.__name__.replace("Adapter", "")
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
    recording_settings: RecordingConfig,
    domain_randomizer: DomainRandomizer | None,
    recorder: LeRobotEpisodeRecorder | None,
    recording_schema,
) -> None:
    ik_disabled_reported = False
    quiet_recording = recorder is not None and not recording_settings.verbose
    control_metrics = ControlMetricsReporter(enabled=recorder is None and not debug_ik)
    button_mapper = ButtonEdgeMapper(recording_settings.buttons)
    recording_state = RecordingLoopState(
        target_episodes=recording_settings.max_episodes,
        recording_active=recording_settings.auto_start_episode and recorder is not None,
    )

    if recorder is not None and recording_settings.verbose:
        _print_recording_waiting_status(recording_state, recorder)

    while isaac_app.is_running():
        rclpy.spin_once(controller_provider, timeout_sec=0.0)
        latest_states = controller_provider.latest()
        session_update = teleop_session.update(latest_states)
        button_events = button_mapper.update(latest_states)

        if recorder is not None and _sync_recording_progress(recording_state, recorder, verbose=not quiet_recording):
            break

        _log_session_events(session_update.events, controller_provider, enabled=not quiet_recording)
        reset_requested = _handle_button_events(
            button_events=button_events,
            isaac_app=isaac_app,
            adapter=adapter,
            teleop_session=teleop_session,
            camera_manager=camera_manager,
            domain_randomizer=domain_randomizer,
            recorder=recorder,
            recording_state=recording_state,
            session_ready=session_update.ready,
            verbose_recording=not quiet_recording,
        )
        if reset_requested:
            isaac_app.step(render=True)
            continue

        if not session_update.ready:
            _handle_not_ready(
                isaac_app,
                controller_provider,
                session_update,
                teleop_session.config,
                verbose=not quiet_recording,
            )
            continue

        current_positions = adapter.get_current_joint_positions()
        if current_positions is None:
            isaac_app.step(render=True)
            continue

        action = adapter.compute_action(session_update.targets)
        _maybe_print_ik_debug(
            adapter,
            teleop_session,
            session_update,
            debug_ik,
            current_positions=current_positions,
            action_positions=action.joint_positions,
        )

        if not ik_enabled and not ik_disabled_reported:
            print("[INFO] IK disabled; holding arm joints while grippers remain responsive")
            ik_disabled_reported = True

        adapter.apply_action(action)
        teleop_session.mark_isaac_apply()
        stamp = controller_provider.get_clock().now().to_msg()
        joint_state_publisher.publish(dof_names, action.joint_positions, stamp=stamp)
        camera_frames = camera_manager.update(
            stamp=stamp,
            return_frames=recorder is not None,
        )

        if recorder is not None and recording_schema is not None and recording_state.recording_active:
            _maybe_record_frame(
                adapter=adapter,
                recorder=recorder,
                recording_settings=recording_settings,
                recording_schema=recording_schema,
                current_positions=current_positions,
                action=action,
                camera_frames=camera_frames or {},
            )
            _maybe_print_recording_frame_progress(
                recording_state,
                recorder,
                verbose=not quiet_recording,
            )
        control_metrics.record(session_update, adapter, controller_provider.get_logger())

        isaac_app.step(render=True)


def _log_session_events(events, controller_provider, *, enabled: bool = True) -> None:
    if not enabled:
        return
    for event in events:
        if event.level == "warning":
            controller_provider.get_logger().warn(event.message)
        else:
            controller_provider.get_logger().info(event.message)


def _handle_button_events(
    *,
    button_events,
    isaac_app: IsaacApp,
    adapter: OpenArmAdapter,
    teleop_session: BimanualTeleopSession,
    camera_manager: CameraManager,
    domain_randomizer: DomainRandomizer | None,
    recorder: LeRobotEpisodeRecorder | None,
    recording_state: RecordingLoopState,
    session_ready: bool,
    verbose_recording: bool = True,
) -> bool:
    if button_events.switch_camera:
        switched_camera = camera_manager.switch_viewport_camera_next()
        if verbose_recording and switched_camera is not None:
            camera_name, camera_path = switched_camera
            print(f"[Camera] Switched to: {camera_name} ({camera_path})")
    if button_events.save_episode and recorder is not None:
        if not recording_state.recording_active:
            if verbose_recording:
                print("[Recording] Ignored save request because no episode is currently recording")
        elif recording_state.awaiting_save_completion or recording_state.awaiting_discard_completion:
            if verbose_recording:
                print("[Recording] Save already in progress; waiting for recorder to finish")
        else:
            recording_state.recording_active = False
            recording_state.awaiting_save_completion = True
            if verbose_recording:
                print(f"[Recording] Save requested for {recording_state.next_episode_label(recorder.diagnostics.saved_episodes)}")
            recorder.save_episode_async(reason="quest_x")
    if button_events.start_episode and recorder is not None:
        if not session_ready:
            if verbose_recording:
                print("[Recording] Wait for calibration to complete before starting an episode")
        elif recording_state.awaiting_save_completion or recording_state.awaiting_discard_completion:
            if verbose_recording:
                print("[Recording] Waiting for previous save/discard to complete before starting the next episode")
        elif recording_state.recording_active:
            if verbose_recording:
                print("[Recording] Episode recording is already active")
        elif recording_state.has_reached_target(recorder.diagnostics.saved_episodes):
            if verbose_recording:
                print(f"[Recording] Episode target already reached: {recording_state.progress_text(recorder.diagnostics.saved_episodes)}")
        else:
            recording_state.recording_active = True
            if verbose_recording:
                print(f"[Recording] Started {recording_state.next_episode_label(recorder.diagnostics.saved_episodes)}")
    if button_events.reset_scene:
        if recorder is not None:
            recording_state.recording_active = False
            recording_state.awaiting_save_completion = False
            recording_state.awaiting_discard_completion = True
            recorder.discard_episode_async(reason="scene_reset")
        if verbose_recording:
            print("[Scene] Reset requested")
        _reset_scene(isaac_app, adapter, teleop_session, domain_randomizer, verbose=verbose_recording)
        return True
    return False


def _sync_recording_progress(
    recording_state: RecordingLoopState,
    recorder: LeRobotEpisodeRecorder,
    *,
    verbose: bool = True,
) -> bool:
    diagnostics = recorder.diagnostics

    if diagnostics.discarded_episodes != recording_state.last_discarded_reported:
        recording_state.last_discarded_reported = diagnostics.discarded_episodes
        recording_state.awaiting_discard_completion = False
        if verbose:
            print("[Recording] Cleared unsaved episode buffer")
            _print_recording_waiting_status(recording_state, recorder)

    if diagnostics.saved_episodes != recording_state.last_saved_reported:
        recording_state.last_saved_reported = diagnostics.saved_episodes
        recording_state.awaiting_save_completion = False
        if verbose:
            print(f"[Recording] Saved {recording_state.progress_text(diagnostics.saved_episodes)}")
        if recording_state.has_reached_target(diagnostics.saved_episodes):
            if verbose:
                print("[Recording] Target episode count reached; stopping teleoperation")
            return True
        if verbose:
            _print_recording_waiting_status(recording_state, recorder)

    return False


def _print_recording_waiting_status(
    recording_state: RecordingLoopState,
    recorder: LeRobotEpisodeRecorder,
) -> None:
    if recording_state.has_reached_target(recorder.diagnostics.saved_episodes):
        print(f"[Recording] Target reached: {recording_state.progress_text(recorder.diagnostics.saved_episodes)}")
        return
    if recording_state.awaiting_save_completion:
        print("[Recording] Waiting for save to finish...")
        return
    if recording_state.awaiting_discard_completion:
        print("[Recording] Waiting for buffer clear to finish...")
        return
    if recording_state.recording_active:
        print(f"[Recording] Active: {recording_state.next_episode_label(recorder.diagnostics.saved_episodes)}")
        return
    print(f"[Recording] Ready to start {recording_state.next_episode_label(recorder.diagnostics.saved_episodes)} with Y")


def _maybe_print_recording_frame_progress(
    recording_state: RecordingLoopState,
    recorder: LeRobotEpisodeRecorder,
    *,
    verbose: bool = True,
) -> None:
    if not verbose:
        return
    now_s = time.monotonic()
    if now_s - recording_state.last_progress_report_s < 3.0:
        return
    recording_state.last_progress_report_s = now_s
    diagnostics = recorder.diagnostics
    print(
        "[Recording] Active "
        f"{recording_state.next_episode_label(diagnostics.saved_episodes)}: "
        f"queued={diagnostics.frames_enqueued}, "
        f"written={diagnostics.frames_written}, "
        f"dropped={diagnostics.frames_dropped_queue_full}, "
        f"queue_depth={diagnostics.queue_depth}"
    )


def _maybe_record_frame(
    *,
    adapter: OpenArmAdapter,
    recorder: LeRobotEpisodeRecorder,
    recording_settings: RecordingConfig,
    recording_schema,
    current_positions,
    action,
    camera_frames: dict[str, object],
) -> None:
    expected_cameras = {camera_spec.camera_name for camera_spec in recording_schema.camera_specs}
    if expected_cameras and expected_cameras.difference(camera_frames):
        return

    _, state_vector = adapter.get_recording_vector(
        vector_config=recording_settings.state,
        current_joint_positions=current_positions,
        commanded_action=action,
    )
    _, action_vector = adapter.get_recording_vector(
        vector_config=recording_settings.action,
        current_joint_positions=current_positions,
        commanded_action=action,
    )

    recorder.enqueue_frame(
        RecordingFrameSnapshot(
            state=state_vector,
            action=action_vector,
            cameras={name: camera_frames[name] for name in expected_cameras},
            task=recording_settings.task,
            monotonic_time_s=time.monotonic(),
        )
    )


def _reset_scene(
    isaac_app: IsaacApp,
    adapter: OpenArmAdapter,
    teleop_session: BimanualTeleopSession,
    domain_randomizer: DomainRandomizer | None = None,
    *,
    verbose: bool = True,
) -> None:
    isaac_app.reset_world()
    adapter.reset_runtime_state()
    teleop_session.reset(preserve_calibration=True)
    if domain_randomizer is not None and domain_randomizer.enabled:
        sample = domain_randomizer.randomize(isaac_app.step)
        if verbose:
            cube_pose = getattr(sample, "cube_pose", None)
            tray_pose = getattr(sample, "tray_pose", None)
            missing_prims = getattr(sample, "missing_prims", ())
            if missing_prims:
                print(
                    "[Scene] Randomization skipped: "
                    f"missing_prims={list(missing_prims)}, "
                    f"light_intensity={sample.light_intensity}"
                )
            elif cube_pose is not None or tray_pose is not None:
                print(
                    "[Scene] Randomized: "
                    f"cube_position={cube_pose[0] if cube_pose else None}, "
                    f"tray_position={tray_pose[0] if tray_pose else None}, "
                    f"light_intensity={sample.light_intensity}"
                )
            else:
                print(
                    "[Scene] Randomized: "
                    f"nuts={sample.nut_count}, bolts={sample.bolt_count}, "
                    f"light_intensity={sample.light_intensity}, floor_color={sample.floor_color}"
                )


def _handle_not_ready(
    isaac_app: IsaacApp,
    controller_provider,
    session_update,
    runtime_config: TeleopSessionConfig,
    *,
    verbose: bool = True,
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
        if verbose and controller_provider.total_pose_count % 30 == 1:
            print(f"[Calibration] Left: {left_status} | Right: {right_status}")
    else:
        if verbose:
            controller_provider.maybe_report_waiting_for_controllers()
    isaac_app.step(render=True)


def _maybe_print_ik_debug(
    adapter: OpenArmAdapter,
    teleop_session: BimanualTeleopSession,
    session_update,
    debug_ik: bool,
    *,
    current_positions,
    action_positions,
) -> None:
    diagnostics = adapter.get_diagnostics()
    frame_count = (
        diagnostics.counters.get("left_ik_success", 0)
        + diagnostics.counters.get("left_ik_fail", 0)
        + diagnostics.counters.get("right_ik_success", 0)
        + diagnostics.counters.get("right_ik_fail", 0)
    )
    if not debug_ik or frame_count == 0 or frame_count % 200 != 0:
        return

    current_fk = adapter.get_debug_end_effector_positions(current_positions)
    commanded_fk = adapter.get_debug_end_effector_positions(action_positions)
    ik_debug = adapter.get_last_ik_debug()
    print("[IK Debug] Controller-to-TCP tracking")
    _print_hand_debug(
        hand="left",
        runtime=teleop_session.left,
        frame_transform=teleop_session.frame_transform,
        pos_scale=teleop_session.config.pos_scale,
        target=session_update.targets.left_ee.position_xyz,
        current_fk=current_fk.get("left"),
        commanded_fk=commanded_fk.get("left"),
        ik_debug=ik_debug.get("left"),
    )
    _print_hand_debug(
        hand="right",
        runtime=teleop_session.right,
        frame_transform=teleop_session.frame_transform,
        pos_scale=teleop_session.config.pos_scale,
        target=session_update.targets.right_ee.position_xyz,
        current_fk=current_fk.get("right"),
        commanded_fk=commanded_fk.get("right"),
        ik_debug=ik_debug.get("right"),
    )


def _print_hand_debug(
    *,
    hand: str,
    runtime,
    frame_transform,
    pos_scale,
    target,
    current_fk,
    commanded_fk,
    ik_debug,
) -> None:
    state = runtime.last_controller_state
    controller_pos = None
    xr_delta = None
    robot_delta = None
    raw_target = None
    if state is not None and state.pose is not None and runtime.reference_position is not None:
        controller_pos = np.asarray(state.pose.position_xyz, dtype=float)
        xr_delta = controller_pos - runtime.reference_position
        robot_delta = frame_transform.position_offset_to_robot(xr_delta)
        raw_target = runtime.home_position + robot_delta * pos_scale

    print(
        f"  {hand}: ctrl={_fmt(controller_pos)} ref={_fmt(runtime.reference_position)} "
        f"xr_delta={_fmt(xr_delta)} robot_delta={_fmt(robot_delta)}"
    )
    print(
        f"  {hand}: raw_target={_fmt(raw_target)} smoothed_target={_fmt(target)} "
        f"current_fk={_fmt(current_fk)} commanded_fk={_fmt(commanded_fk)}"
    )
    print(
        f"  {hand}: err_current={_fmt_error(target, current_fk)} "
        f"err_commanded={_fmt_error(target, commanded_fk)}"
    )
    print(f"  {hand}: ik_status={_fmt_mapping(ik_debug)}")


def _fmt(values) -> str:
    if values is None:
        return "None"
    return np.array2string(
        np.asarray(values, dtype=float).reshape(-1),
        precision=4,
        suppress_small=True,
    )


def _fmt_error(target, measured) -> str:
    if target is None or measured is None:
        return "None"
    error = np.asarray(target, dtype=float).reshape(3) - np.asarray(measured, dtype=float).reshape(3)
    return f"{_fmt(error)} |norm={np.linalg.norm(error):.4f}m"


def _fmt_mapping(values) -> str:
    if not values:
        return "None"
    return " ".join(f"{key}={value}" for key, value in values.items())


def _print_session_statistics(
    adapter: OpenArmAdapter,
    camera_manager: CameraManager,
    recorder: LeRobotEpisodeRecorder | None,
) -> None:
    print("\n" + "=" * 60)
    diagnostics = adapter.get_diagnostics()
    print("Session Statistics:")
    print(
        "  Left Arm  - IK Success: "
        f"{diagnostics.counters.get('left_ik_success', 0)}, "
        f"IK Fail: {diagnostics.counters.get('left_ik_fail', 0)}, "
        f"Orientation Fallback: {diagnostics.counters.get('left_orientation_fallback', 0)}"
    )
    print(
        "  Right Arm - IK Success: "
        f"{diagnostics.counters.get('right_ik_success', 0)}, "
        f"IK Fail: {diagnostics.counters.get('right_ik_fail', 0)}, "
        f"Orientation Fallback: {diagnostics.counters.get('right_orientation_fallback', 0)}"
    )
    print(
        "  Camera    - Published: "
        f"{camera_manager.diagnostics.published_frames}, "
        f"Dropped: {camera_manager.diagnostics.dropped_frames}, "
        f"Errors: {camera_manager.diagnostics.errors}"
    )
    if recorder is not None:
        print(
            "  Recording - Frames queued: "
            f"{recorder.diagnostics.frames_enqueued}, "
            f"written: {recorder.diagnostics.frames_written}, "
            f"dropped: {recorder.diagnostics.frames_dropped_queue_full}, "
            f"saved episodes: {recorder.diagnostics.saved_episodes}, "
            f"discarded episodes: {recorder.diagnostics.discarded_episodes}"
        )
        if recorder.diagnostics.last_error is not None:
            print(f"  Recording - Last error: {recorder.diagnostics.last_error}")
    print("=" * 60)
