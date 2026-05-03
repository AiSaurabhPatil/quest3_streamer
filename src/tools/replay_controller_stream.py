#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import time


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.dirname(SCRIPT_DIR)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from tools.controller_stream import load_controller_stream  # noqa: E402


def import_ros_interfaces():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import Joy

    return rclpy, Node, PoseStamped, Joy


def build_replay_node_class(Node, PoseStamped, Joy):
    class ControllerStreamReplay(Node):
        def __init__(self):
            super().__init__("controller_stream_replay")
            self.pub_left_pose = self.create_publisher(PoseStamped, "/quest/left_hand/pose", 10)
            self.pub_right_pose = self.create_publisher(PoseStamped, "/quest/right_hand/pose", 10)
            self.pub_left_input = self.create_publisher(Joy, "/quest/left_hand/inputs", 10)
            self.pub_right_input = self.create_publisher(Joy, "/quest/right_hand/inputs", 10)

        def publish_controller_state(self, state) -> None:
            if state is None:
                return

            stamp = self.get_clock().now().to_msg()
            if state.has_valid_pose:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = stamp
                pose_msg.header.frame_id = "quest_world"
                pose_msg.pose.position.x = float(state.pose.position_xyz[0])
                pose_msg.pose.position.y = float(state.pose.position_xyz[1])
                pose_msg.pose.position.z = float(state.pose.position_xyz[2])
                pose_msg.pose.orientation.x = float(state.pose.orientation_xyzw[0])
                pose_msg.pose.orientation.y = float(state.pose.orientation_xyzw[1])
                pose_msg.pose.orientation.z = float(state.pose.orientation_xyzw[2])
                pose_msg.pose.orientation.w = float(state.pose.orientation_xyzw[3])
                if state.hand == "left":
                    self.pub_left_pose.publish(pose_msg)
                else:
                    self.pub_right_pose.publish(pose_msg)

            joy_msg = Joy()
            joy_msg.header.stamp = stamp
            joy_msg.axes = [
                float(state.axes.trigger),
                float(state.axes.squeeze),
                float(state.axes.thumbstick_x),
                float(state.axes.thumbstick_y),
            ]
            joy_msg.buttons = [
                int(state.buttons.primary),
                int(state.buttons.secondary),
                int(state.buttons.menu),
                int(state.buttons.stick_click),
            ]
            if state.hand == "left":
                self.pub_left_input.publish(joy_msg)
            else:
                self.pub_right_input.publish(joy_msg)

    return ControllerStreamReplay


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay a recorded Quest controller JSONL stream.")
    parser.add_argument("input", help="Path to the recorded JSONL stream.")
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Replay speed multiplier when using recorded timing.",
    )
    parser.add_argument(
        "--fixed-rate-hz",
        type=float,
        default=None,
        help="Ignore recorded timing and publish at a fixed frame rate.",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Loop the stream until interrupted.",
    )
    return parser.parse_args(argv)


def replay_once(node, frames, *, rclpy, speed: float, fixed_rate_hz: float | None) -> int:
    if not frames:
        node.get_logger().warn("No frames found in controller stream.")
        return 0

    if fixed_rate_hz is not None and fixed_rate_hz <= 0.0:
        raise ValueError("--fixed-rate-hz must be positive")
    if fixed_rate_hz is None and speed <= 0.0:
        raise ValueError("--speed must be positive")

    start_wall_s = time.monotonic()
    first_frame_t = float(frames[0].t)
    published = 0

    for index, frame in enumerate(frames):
        if fixed_rate_hz is not None:
            target_elapsed_s = index / fixed_rate_hz
        else:
            target_elapsed_s = max(0.0, (float(frame.t) - first_frame_t) / speed)

        while True:
            remaining_s = target_elapsed_s - (time.monotonic() - start_wall_s)
            if remaining_s <= 0.0:
                break
            time.sleep(min(remaining_s, 0.01))
            rclpy.spin_once(node, timeout_sec=0.0)

        states = frame.controller_states(frame.t)
        node.publish_controller_state(states["left"])
        node.publish_controller_state(states["right"])
        published += 1

    node.get_logger().info(f"Replayed {published} frames")
    return published


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    frames = load_controller_stream(Path(args.input).expanduser().resolve())

    rclpy, Node, PoseStamped, Joy = import_ros_interfaces()
    ReplayNode = build_replay_node_class(Node, PoseStamped, Joy)

    rclpy.init()
    node = ReplayNode()
    try:
        while rclpy.ok():
            replay_once(
                node,
                frames,
                rclpy=rclpy,
                speed=args.speed,
                fixed_rate_hz=args.fixed_rate_hz,
            )
            if not args.loop:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
