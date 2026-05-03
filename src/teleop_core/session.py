from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Mapping
import time

import numpy as np

from .calibration import BimanualCalibration, CalibrationStatus, HandCalibration
from .controller_state import ControllerState
from .filters import OrientationSlerp, PositionEMA
from .frame_transforms import (
    DEFAULT_TOOL_ROTATION_CORRECTION,
    DEFAULT_VR_TO_ROBOT,
    FrameTransform,
)
from .retargeting import (
    BimanualTeleopTargets,
    EndEffectorTarget,
    GripperTarget,
    SingleArmTeleopTargets,
)
from .safety import TargetSafety, TargetSafetyConfig, WorkspaceBounds


def _as_vector(values, size: int, fallback) -> np.ndarray:
    array = np.asarray(values, dtype=float).reshape(-1)
    if array.size != size:
        array = np.asarray(fallback, dtype=float)
    return array.copy()


@dataclass
class TeleopSessionConfig:
    pos_scale: np.ndarray = field(default_factory=lambda: np.ones(3, dtype=float))
    robot_workspace_center: np.ndarray = field(
        default_factory=lambda: np.array([0.3, 0.0, 0.3], dtype=float)
    )
    left_arm_offset: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.15, 0.0], dtype=float)
    )
    right_arm_offset: np.ndarray = field(
        default_factory=lambda: np.array([0.0, -0.15, 0.0], dtype=float)
    )
    smoothing: float = 0.9
    position_alpha: float | None = None
    orientation_alpha: float | None = None
    gripper_threshold: float = 0.5
    calibration_samples: int = 30
    deadman_timeout_s: float = 0.25
    hard_timeout_s: float = 1.0
    max_target_jump_m: float | None = None
    workspace_bounds: WorkspaceBounds | None = None
    enable_prediction: bool = False
    prediction_horizon_s: float = 0.05
    jitter_buffer_frames: int = 0

    def __post_init__(self):
        if np.isscalar(self.pos_scale):
            self.pos_scale = np.full(3, float(self.pos_scale), dtype=float)
        else:
            self.pos_scale = _as_vector(self.pos_scale, 3, np.ones(3, dtype=float))
        self.robot_workspace_center = _as_vector(
            self.robot_workspace_center,
            3,
            np.array([0.3, 0.0, 0.3], dtype=float),
        )
        self.left_arm_offset = _as_vector(
            self.left_arm_offset,
            3,
            np.array([0.0, 0.15, 0.0], dtype=float),
        )
        self.right_arm_offset = _as_vector(
            self.right_arm_offset,
            3,
            np.array([0.0, -0.15, 0.0], dtype=float),
        )
        self.smoothing = float(np.clip(self.smoothing, 0.0, 1.0))
        if self.position_alpha is None:
            self.position_alpha = self.smoothing
        if self.orientation_alpha is None:
            self.orientation_alpha = self.smoothing
        self.position_alpha = float(np.clip(self.position_alpha, 0.0, 1.0))
        self.orientation_alpha = float(np.clip(self.orientation_alpha, 0.0, 1.0))
        self.gripper_threshold = float(self.gripper_threshold)
        self.calibration_samples = max(1, int(self.calibration_samples))
        self.deadman_timeout_s = max(0.0, float(self.deadman_timeout_s))
        self.hard_timeout_s = max(self.deadman_timeout_s, float(self.hard_timeout_s))
        if self.max_target_jump_m is not None:
            self.max_target_jump_m = max(0.0, float(self.max_target_jump_m))
        self.enable_prediction = bool(self.enable_prediction)
        self.prediction_horizon_s = max(0.0, float(self.prediction_horizon_s))
        self.jitter_buffer_frames = max(0, int(self.jitter_buffer_frames))

    @classmethod
    def from_dict(cls, values: Mapping[str, object]) -> "TeleopSessionConfig":
        return cls(**dict(values))


@dataclass
class SessionEvent:
    level: str
    message: str
    hand: str | None = None


@dataclass
class HandSessionState:
    hand: str
    calibrated: bool
    received_pose_count: int
    stale: bool
    hard_timeout_active: bool
    last_age_ms: float | None
    sequence: int | None = None
    source_timestamp: float | None = None
    client_epoch_ms: float | None = None
    ingress_receive_epoch_ms: float | None = None
    remote_receive_epoch_ms: float | None = None
    ros_publish_epoch_ms: float | None = None
    control_receive_epoch_ms: float | None = None
    isaac_apply_epoch_ms: float | None = None
    reference_position: np.ndarray | None = None
    home_position: np.ndarray | None = None


@dataclass
class TeleopSessionUpdate:
    ready: bool
    targets: BimanualTeleopTargets
    calibration_status: CalibrationStatus
    left_state: HandSessionState
    right_state: HandSessionState
    events: list[SessionEvent] = field(default_factory=list)


@dataclass
class SingleArmTeleopSessionUpdate:
    ready: bool
    targets: SingleArmTeleopTargets
    calibration_samples: int
    hand_state: HandSessionState
    events: list[SessionEvent] = field(default_factory=list)


class _HandRuntime:
    def __init__(
        self,
        hand: str,
        home_position: np.ndarray,
        config: TeleopSessionConfig,
        calibration: HandCalibration | None = None,
    ):
        self.hand = hand
        self.home_position = np.asarray(home_position, dtype=float).reshape(3)
        self.calibration = calibration or HandCalibration(config.calibration_samples)
        self.reference_position: np.ndarray | None = None
        self.target_pos = self.home_position.copy()
        self.target_rot = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.smoothed_pos = self.home_position.copy()
        self.smoothed_rot = self.target_rot.copy()
        self.target_velocity_mps = np.zeros(3, dtype=float)
        self.calibrated = False
        self.received_pose_count = 0
        self.last_controller_state: ControllerState | None = None
        self.last_processed_pose_marker: tuple[int, float, float] | None = None
        self.last_target_update_s: float | None = None
        self.last_stale_report_s = 0.0
        self.hard_timeout_active = False
        self.position_filter = (
            PositionEMA(config.position_alpha) if config.position_alpha > 0.0 else None
        )
        self.orientation_filter = (
            OrientationSlerp(config.orientation_alpha)
            if config.orientation_alpha > 0.0
            else None
        )
        self.safety = TargetSafety(
            TargetSafetyConfig(
                stale_timeout_s=config.deadman_timeout_s,
                max_translation_step_m=config.max_target_jump_m,
                workspace_bounds=config.workspace_bounds,
            )
        )
        self.pending_states: deque[ControllerState] = deque(
            maxlen=max(1, config.jitter_buffer_frames + 1)
        )

    def reset_filters(self) -> None:
        if self.position_filter is not None:
            self.position_filter.reset(self.home_position)
        if self.orientation_filter is not None:
            self.orientation_filter.reset(self.target_rot)


class BimanualTeleopSession:
    def __init__(
        self,
        config: TeleopSessionConfig,
        frame_transform: FrameTransform | None = None,
        calibration: BimanualCalibration | None = None,
    ):
        self.config = config
        self.frame_transform = frame_transform or FrameTransform(
            matrix_vr_to_robot=DEFAULT_VR_TO_ROBOT,
            tool_rotation_correction=DEFAULT_TOOL_ROTATION_CORRECTION,
        )
        self.calibration = calibration or BimanualCalibration(config.calibration_samples)
        self.left = _HandRuntime(
            hand="left",
            home_position=config.robot_workspace_center + config.left_arm_offset,
            config=config,
            calibration=self.calibration.left,
        )
        self.right = _HandRuntime(
            hand="right",
            home_position=config.robot_workspace_center + config.right_arm_offset,
            config=config,
            calibration=self.calibration.right,
        )

    def update(
        self,
        controller_states: Mapping[str, ControllerState | None],
        now_s: float | None = None,
    ) -> TeleopSessionUpdate:
        if now_s is None:
            now_s = time.monotonic()

        events: list[SessionEvent] = []
        self._ingest_controller_state(self.left, controller_states.get("left"), events)
        self._ingest_controller_state(self.right, controller_states.get("right"), events)
        self._update_deadman(self.left, now_s, events)
        self._update_deadman(self.right, now_s, events)

        targets = BimanualTeleopTargets(
            left_ee=EndEffectorTarget(
                position_xyz=self._smoothed_position(self.left, now_s),
                orientation_wxyz=self._smoothed_orientation(self.left),
                valid=self.left.calibrated,
            ),
            right_ee=EndEffectorTarget(
                position_xyz=self._smoothed_position(self.right, now_s),
                orientation_wxyz=self._smoothed_orientation(self.right),
                valid=self.right.calibrated,
            ),
            left_gripper=self._gripper_target(self.left),
            right_gripper=self._gripper_target(self.right),
        )

        return TeleopSessionUpdate(
            ready=self.left.calibrated and self.right.calibrated,
            targets=targets,
            calibration_status=self.calibration.status,
            left_state=self._snapshot(self.left, now_s),
            right_state=self._snapshot(self.right, now_s),
            events=events,
        )

    def _ingest_controller_state(
        self,
        runtime: _HandRuntime,
        controller_state: ControllerState | None,
        events: list[SessionEvent],
    ) -> None:
        if controller_state is None:
            return

        controller_state.control_receive_epoch_ms = time.time() * 1000.0
        runtime.last_controller_state = controller_state
        if not controller_state.has_valid_pose:
            return

        marker = self._pose_marker(controller_state)
        if marker == runtime.last_processed_pose_marker:
            return
        runtime.last_processed_pose_marker = marker
        runtime.pending_states.append(controller_state)
        if len(runtime.pending_states) <= self.config.jitter_buffer_frames:
            return

        controller_state = runtime.pending_states.popleft()
        runtime.received_pose_count += 1

        if not runtime.calibrated:
            if runtime.calibration.add_sample(controller_state.pose.position_xyz):
                runtime.reference_position = runtime.calibration.reference_position
                runtime.target_pos = runtime.home_position.copy()
                runtime.smoothed_pos = runtime.home_position.copy()
                runtime.target_rot = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
                runtime.smoothed_rot = runtime.target_rot.copy()
                runtime.target_velocity_mps = np.zeros(3, dtype=float)
                runtime.calibrated = True
                runtime.safety.apply_position(runtime.home_position)
                runtime.reset_filters()
                events.append(
                    SessionEvent(
                        level="info",
                        hand=runtime.hand,
                        message=(
                            f"{runtime.hand.upper()} ARM CALIBRATION COMPLETE\n"
                            f"  Home position set to: {runtime.home_position}"
                        ),
                    )
                )
            return

        reference_position = runtime.reference_position
        if reference_position is None:
            return

        xr_offset = controller_state.pose.position_xyz - reference_position
        robot_offset = self.frame_transform.position_offset_to_robot(xr_offset)
        robot_pos = robot_offset * self.config.pos_scale + runtime.home_position
        robot_rot = self.frame_transform.orientation_xyzw_to_robot_wxyz(
            controller_state.pose.orientation_xyzw
        )

        dt_s = None
        if runtime.last_target_update_s is not None:
            dt_s = controller_state.receive_time_s - runtime.last_target_update_s

        safe_pos, accepted = runtime.safety.apply_position(
            robot_pos,
            previous_xyz=runtime.target_pos,
            dt_s=dt_s,
        )
        if accepted:
            if dt_s is not None and dt_s > 1e-6:
                runtime.target_velocity_mps = (safe_pos - runtime.target_pos) / dt_s
            runtime.target_pos = safe_pos
            runtime.target_rot = robot_rot

        runtime.last_target_update_s = controller_state.receive_time_s

    def _update_deadman(
        self,
        runtime: _HandRuntime,
        now_s: float,
        events: list[SessionEvent],
    ) -> None:
        if not runtime.calibrated or runtime.last_controller_state is None:
            return

        age_s = runtime.last_controller_state.age_s(now_s)
        if age_s <= self.config.deadman_timeout_s:
            if runtime.hard_timeout_active:
                runtime.hard_timeout_active = False
                events.append(
                    SessionEvent(
                        level="info",
                        hand=runtime.hand,
                        message=f"{runtime.hand.upper()} controller data restored",
                    )
                )
            return

        if now_s - runtime.last_stale_report_s > 1.0:
            events.append(
                SessionEvent(
                    level="warning",
                    hand=runtime.hand,
                    message=(
                        f"{runtime.hand.upper()} controller stale for {age_s * 1000.0:.0f} ms; "
                        "holding last valid IK target"
                    ),
                )
            )
            runtime.last_stale_report_s = now_s

        if age_s > self.config.hard_timeout_s and not runtime.hard_timeout_active:
            runtime.hard_timeout_active = True
            events.append(
                SessionEvent(
                    level="warning",
                    hand=runtime.hand,
                    message=f"{runtime.hand.upper()} controller hard timeout; opening gripper",
                )
            )

    def _gripper_target(self, runtime: _HandRuntime) -> GripperTarget:
        controller_state = runtime.last_controller_state
        analog_value = 0.0
        closed = False
        if controller_state is not None:
            analog_value = max(
                float(controller_state.axes.trigger),
                float(controller_state.axes.squeeze),
            )
            closed = analog_value > self.config.gripper_threshold

        if runtime.hard_timeout_active:
            closed = False

        return GripperTarget(closed=closed, analog_value=analog_value)

    def _predicted_position(self, runtime: _HandRuntime, now_s: float) -> np.ndarray:
        if (
            not self.config.enable_prediction
            or runtime.last_target_update_s is None
            or runtime.last_controller_state is None
            or runtime.hard_timeout_active
            or runtime.last_controller_state.age_s(now_s) > self.config.deadman_timeout_s
        ):
            return runtime.target_pos.copy()

        horizon_s = min(
            self.config.prediction_horizon_s,
            max(0.0, now_s - runtime.last_target_update_s),
        )
        if horizon_s <= 0.0:
            return runtime.target_pos.copy()

        predicted = runtime.target_pos + runtime.target_velocity_mps * horizon_s
        if self.config.workspace_bounds is not None:
            predicted = self.config.workspace_bounds.clamp(predicted)
        return predicted

    def _smoothed_position(self, runtime: _HandRuntime, now_s: float) -> np.ndarray:
        target_pos = self._predicted_position(runtime, now_s)
        if runtime.position_filter is None:
            runtime.smoothed_pos = target_pos.copy()
        else:
            runtime.smoothed_pos = runtime.position_filter.update(target_pos)
        return runtime.smoothed_pos.copy()

    def _smoothed_orientation(self, runtime: _HandRuntime) -> np.ndarray:
        if runtime.orientation_filter is None:
            runtime.smoothed_rot = runtime.target_rot.copy()
        else:
            runtime.smoothed_rot = runtime.orientation_filter.update(runtime.target_rot)
        return runtime.smoothed_rot.copy()

    def _snapshot(self, runtime: _HandRuntime, now_s: float) -> HandSessionState:
        age_ms = None
        stale = False
        if runtime.last_controller_state is not None:
            controller_state = runtime.last_controller_state
            age_ms = runtime.last_controller_state.age_s(now_s) * 1000.0
            stale = (
                runtime.calibrated
                and age_ms > self.config.deadman_timeout_s * 1000.0
            )
        else:
            controller_state = None

        return HandSessionState(
            hand=runtime.hand,
            calibrated=runtime.calibrated,
            received_pose_count=runtime.received_pose_count,
            stale=stale,
            hard_timeout_active=runtime.hard_timeout_active,
            last_age_ms=age_ms,
            sequence=None if controller_state is None else int(controller_state.sequence),
            source_timestamp=None
            if controller_state is None
            else float(controller_state.source_timestamp),
            client_epoch_ms=None if controller_state is None else controller_state.client_epoch_ms,
            ingress_receive_epoch_ms=None
            if controller_state is None
            else controller_state.ingress_receive_epoch_ms,
            remote_receive_epoch_ms=None
            if controller_state is None
            else controller_state.remote_receive_epoch_ms,
            ros_publish_epoch_ms=None
            if controller_state is None
            else controller_state.ros_publish_epoch_ms,
            control_receive_epoch_ms=None
            if controller_state is None
            else controller_state.control_receive_epoch_ms,
            isaac_apply_epoch_ms=None
            if controller_state is None
            else controller_state.isaac_apply_epoch_ms,
            reference_position=None
            if runtime.reference_position is None
            else runtime.reference_position.copy(),
            home_position=runtime.home_position.copy(),
        )

    @staticmethod
    def _pose_marker(controller_state: ControllerState) -> tuple[int, float, float]:
        return (
            int(controller_state.sequence),
            float(controller_state.source_timestamp),
            float(controller_state.receive_time_s),
        )

    def mark_isaac_apply(self, apply_epoch_ms: float | None = None) -> None:
        apply_epoch_ms = time.time() * 1000.0 if apply_epoch_ms is None else apply_epoch_ms
        for runtime in (self.left, self.right):
            if runtime.last_controller_state is not None:
                runtime.last_controller_state.isaac_apply_epoch_ms = float(apply_epoch_ms)


class SingleArmTeleopSession(BimanualTeleopSession):
    def __init__(
        self,
        config: TeleopSessionConfig,
        *,
        hand: str = "right",
        home_position: np.ndarray | None = None,
        frame_transform: FrameTransform | None = None,
        calibration: HandCalibration | None = None,
    ):
        self.config = config
        self.frame_transform = frame_transform or FrameTransform(
            matrix_vr_to_robot=DEFAULT_VR_TO_ROBOT,
            tool_rotation_correction=DEFAULT_TOOL_ROTATION_CORRECTION,
        )
        self.hand = str(hand)
        self.calibration = calibration or HandCalibration(config.calibration_samples)
        self.arm = _HandRuntime(
            hand=self.hand,
            home_position=(
                config.robot_workspace_center
                if home_position is None
                else np.asarray(home_position, dtype=float)
            ),
            config=config,
            calibration=self.calibration,
        )

    def update(
        self,
        controller_state: ControllerState | None,
        now_s: float | None = None,
    ) -> SingleArmTeleopSessionUpdate:
        if now_s is None:
            now_s = time.monotonic()

        events: list[SessionEvent] = []
        self._ingest_controller_state(self.arm, controller_state, events)
        self._update_deadman(self.arm, now_s, events)

        targets = SingleArmTeleopTargets(
            ee_target=EndEffectorTarget(
                position_xyz=self._smoothed_position(self.arm, now_s),
                orientation_wxyz=self._smoothed_orientation(self.arm),
                valid=self.arm.calibrated,
            ),
            gripper_target=self._gripper_target(self.arm),
        )
        return SingleArmTeleopSessionUpdate(
            ready=self.arm.calibrated,
            targets=targets,
            calibration_samples=self.calibration.sample_count,
            hand_state=self._snapshot(self.arm, now_s),
            events=events,
        )

    def mark_isaac_apply(self, apply_epoch_ms: float | None = None) -> None:
        apply_epoch_ms = time.time() * 1000.0 if apply_epoch_ms is None else apply_epoch_ms
        if self.arm.last_controller_state is not None:
            self.arm.last_controller_state.isaac_apply_epoch_ms = float(apply_epoch_ms)
