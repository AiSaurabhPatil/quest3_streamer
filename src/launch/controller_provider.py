from __future__ import annotations

import time

import numpy as np

from src.quest_ingress import parse_ros_frame_id_metadata
from src.teleop_core import (
    ControllerAxes,
    ControllerButtons,
    ControllerPose,
    ControllerState,
)


def ros_stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def import_ros_interfaces():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import Joy

    return rclpy, Node, PoseStamped, Joy


def _metadata_from_header(header) -> dict[str, float | int]:
    _, metadata = parse_ros_frame_id_metadata(getattr(header, "frame_id", ""))
    return metadata


def _state_from_pose_msg(
    msg,
    hand: str,
    axes: ControllerAxes,
    buttons: ControllerButtons,
) -> ControllerState:
    metadata = _metadata_from_header(msg.header)
    stamp_s = ros_stamp_to_seconds(msg.header.stamp)
    return ControllerState(
        hand=hand,
        sequence=int(metadata.get("seq", -1)),
        source_timestamp=float(metadata.get("ts", stamp_s)),
        receive_time_s=time.monotonic(),
        pose=ControllerPose(
            position_xyz=np.array(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                dtype=float,
            ),
            orientation_xyzw=np.array(
                [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ],
                dtype=float,
            ),
        ),
        axes=axes,
        buttons=buttons,
        client_epoch_ms=_optional_float(metadata.get("client")),
        ingress_receive_epoch_ms=_optional_float(metadata.get("ingress")),
        remote_receive_epoch_ms=_optional_float(metadata.get("remote")),
        ros_publish_epoch_ms=_optional_float(metadata.get("ros")),
        control_receive_epoch_ms=time.time() * 1000.0,
        isaac_apply_epoch_ms=_optional_float(metadata.get("isaac")),
    )


def _optional_float(value) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def build_controller_provider_class(
    Node,
    PoseStamped,
    Joy,
    *,
    node_name: str,
    hands: tuple[str, ...] = ("left", "right"),
):
    class QuestControllerProvider(Node):
        """ROS2 node that tracks the latest Quest controller state for each hand."""

        def __init__(self):
            super().__init__(node_name)
            self._hands = tuple(hands)
            self._latest_states = {hand: None for hand in self._hands}
            self._axes = {hand: ControllerAxes() for hand in self._hands}
            self._buttons = {hand: ControllerButtons() for hand in self._hands}
            self._pose_counts = {hand: 0 for hand in self._hands}
            self._last_waiting_print_s = 0.0
            self._subscriptions = []

            for hand in self._hands:
                self._subscriptions.append(
                    self.create_subscription(
                        PoseStamped,
                        f"/quest/{hand}_hand/pose",
                        lambda msg, hand=hand: self.pose_callback(msg, hand),
                        10,
                    )
                )
                self._subscriptions.append(
                    self.create_subscription(
                        Joy,
                        f"/quest/{hand}_hand/inputs",
                        lambda msg, hand=hand: self.input_callback(msg, hand),
                        10,
                    )
                )

            self.get_logger().info("QuestControllerProvider initialized")
            self.get_logger().info("Waiting for Quest controller data...")

        def pose_callback(self, msg, hand: str):
            self._pose_counts[hand] += 1
            self._latest_states[hand] = _state_from_pose_msg(
                msg,
                hand,
                self._axes[hand],
                self._buttons[hand],
            )

        def input_callback(self, msg, hand: str):
            self._axes[hand] = ControllerAxes(
                trigger=msg.axes[0] if len(msg.axes) > 0 else 0.0,
                squeeze=msg.axes[1] if len(msg.axes) > 1 else 0.0,
                thumbstick_x=msg.axes[2] if len(msg.axes) > 2 else 0.0,
                thumbstick_y=msg.axes[3] if len(msg.axes) > 3 else 0.0,
            )

            primary_pressed = len(msg.buttons) > 0 and msg.buttons[0] == 1
            self._buttons[hand] = ControllerButtons(
                primary=primary_pressed,
                secondary=(len(msg.buttons) > 1 and msg.buttons[1] == 1),
                menu=(len(msg.buttons) > 2 and msg.buttons[2] == 1),
                stick_click=(len(msg.buttons) > 3 and msg.buttons[3] == 1),
            )

            previous_state = self._latest_states[hand]
            if previous_state is None:
                return

            self._latest_states[hand] = ControllerState(
                hand=hand,
                sequence=previous_state.sequence,
                source_timestamp=previous_state.source_timestamp,
                receive_time_s=previous_state.receive_time_s,
                pose=previous_state.pose,
                axes=self._axes[hand],
                buttons=self._buttons[hand],
                client_epoch_ms=previous_state.client_epoch_ms,
                ingress_receive_epoch_ms=previous_state.ingress_receive_epoch_ms,
                remote_receive_epoch_ms=previous_state.remote_receive_epoch_ms,
                ros_publish_epoch_ms=previous_state.ros_publish_epoch_ms,
                control_receive_epoch_ms=previous_state.control_receive_epoch_ms,
                isaac_apply_epoch_ms=previous_state.isaac_apply_epoch_ms,
            )

        def latest(self):
            return dict(self._latest_states)

        def latest_hand(self, hand: str):
            return self._latest_states.get(hand)

        @property
        def camera_switch_pressed(self) -> bool:
            return any(buttons.primary for buttons in self._buttons.values())

        @property
        def total_pose_count(self) -> int:
            return sum(self._pose_counts.values())

        def maybe_report_waiting_for_controllers(self):
            current_time_s = time.time()
            if current_time_s - self._last_waiting_print_s <= 2.0:
                return
            topics = ", ".join(f"/quest/{hand}_hand/pose" for hand in self._hands)
            print("[Waiting] No Quest controller data received yet. Is the Quest ROS2 bridge running?")
            print("          Check: ros2 topic list | grep quest")
            print(f"          Expected topics: {topics}")
            self._last_waiting_print_s = current_time_s

    return QuestControllerProvider
