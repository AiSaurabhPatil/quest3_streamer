#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.dirname(SCRIPT_DIR)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from teleop_core import (  # noqa: E402
    ControllerAxes,
    ControllerButtons,
    ControllerPose,
    ControllerState,
)
from tools.controller_stream import ControllerStreamFrame  # noqa: E402


def import_ros_interfaces():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import Joy

    return rclpy, Node, PoseStamped, Joy


def ros_stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def default_output_path(project_root: str) -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return Path(project_root) / "recordings" / "controller_streams" / f"{timestamp}.jsonl"


def build_recorder_node_class(Node, PoseStamped, Joy):
    class ControllerStreamRecorder(Node):
        def __init__(self, output_path: Path):
            super().__init__("controller_stream_recorder")
            self.output_path = output_path
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self._handle = self.output_path.open("w", encoding="utf-8")
            self._start_time_s = time.monotonic()
            self._latest_states = {"left": None, "right": None}
            self._axes = {"left": ControllerAxes(), "right": ControllerAxes()}
            self._buttons = {"left": ControllerButtons(), "right": ControllerButtons()}
            self._frames_written = 0
            self._last_log_s = self._start_time_s

            self.create_subscription(
                PoseStamped,
                "/quest/left_hand/pose",
                lambda msg: self.pose_callback(msg, "left"),
                10,
            )
            self.create_subscription(
                Joy,
                "/quest/left_hand/inputs",
                lambda msg: self.input_callback(msg, "left"),
                10,
            )
            self.create_subscription(
                PoseStamped,
                "/quest/right_hand/pose",
                lambda msg: self.pose_callback(msg, "right"),
                10,
            )
            self.create_subscription(
                Joy,
                "/quest/right_hand/inputs",
                lambda msg: self.input_callback(msg, "right"),
                10,
            )

            self.get_logger().info(f"Recording controller stream to {self.output_path}")

        @property
        def frames_written(self) -> int:
            return self._frames_written

        def close(self) -> None:
            if not self._handle.closed:
                self._handle.close()

        def pose_callback(self, msg, hand: str) -> None:
            self._latest_states[hand] = ControllerState(
                hand=hand,
                sequence=-1,
                source_timestamp=ros_stamp_to_seconds(msg.header.stamp),
                receive_time_s=time.monotonic(),
                pose=ControllerPose(
                    position_xyz=[
                        msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z,
                    ],
                    orientation_xyzw=[
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w,
                    ],
                ),
                axes=self._axes[hand],
                buttons=self._buttons[hand],
            )
            self._write_snapshot()

        def input_callback(self, msg, hand: str) -> None:
            self._axes[hand] = ControllerAxes(
                trigger=msg.axes[0] if len(msg.axes) > 0 else 0.0,
                squeeze=msg.axes[1] if len(msg.axes) > 1 else 0.0,
                thumbstick_x=msg.axes[2] if len(msg.axes) > 2 else 0.0,
                thumbstick_y=msg.axes[3] if len(msg.axes) > 3 else 0.0,
            )
            self._buttons[hand] = ControllerButtons(
                primary=(len(msg.buttons) > 0 and msg.buttons[0] == 1),
                secondary=(len(msg.buttons) > 1 and msg.buttons[1] == 1),
                menu=(len(msg.buttons) > 2 and msg.buttons[2] == 1),
                stick_click=(len(msg.buttons) > 3 and msg.buttons[3] == 1),
            )

            previous_state = self._latest_states[hand]
            if previous_state is None:
                self._latest_states[hand] = ControllerState(
                    hand=hand,
                    receive_time_s=time.monotonic(),
                    axes=self._axes[hand],
                    buttons=self._buttons[hand],
                )
            else:
                self._latest_states[hand] = ControllerState(
                    hand=hand,
                    sequence=previous_state.sequence,
                    source_timestamp=previous_state.source_timestamp,
                    receive_time_s=previous_state.receive_time_s,
                    pose=previous_state.pose,
                    axes=self._axes[hand],
                    buttons=self._buttons[hand],
                )
            self._write_snapshot()

        def _write_snapshot(self) -> None:
            frame = ControllerStreamFrame(
                t=time.monotonic() - self._start_time_s,
                left=self._latest_states["left"],
                right=self._latest_states["right"],
            )
            self._handle.write(json.dumps(frame.to_mapping(), separators=(",", ":")))
            self._handle.write("\n")
            self._handle.flush()
            self._frames_written += 1

            now_s = time.monotonic()
            if now_s - self._last_log_s >= 2.0:
                self.get_logger().info(
                    f"Recorded {self._frames_written} frames to {self.output_path.name}"
                )
                self._last_log_s = now_s

    return ControllerStreamRecorder


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    project_root = os.path.dirname(SRC_DIR)
    parser = argparse.ArgumentParser(description="Record Quest controller states to JSONL.")
    parser.add_argument(
        "--output",
        default=str(default_output_path(project_root)),
        help="Path to the output JSONL file.",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=None,
        help="Optional max recording duration in seconds.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    rclpy, Node, PoseStamped, Joy = import_ros_interfaces()
    RecorderNode = build_recorder_node_class(Node, PoseStamped, Joy)

    should_stop = False

    def request_stop(*_args):
        nonlocal should_stop
        should_stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rclpy.init()
    recorder = RecorderNode(Path(args.output).expanduser().resolve())
    start_time_s = time.monotonic()

    try:
        while rclpy.ok() and not should_stop:
            rclpy.spin_once(recorder, timeout_sec=0.1)
            if args.duration_s is not None and (time.monotonic() - start_time_s) >= args.duration_s:
                break
    finally:
        recorder.get_logger().info(
            f"Finished recording {recorder.frames_written} frames to {recorder.output_path}"
        )
        recorder.close()
        recorder.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
