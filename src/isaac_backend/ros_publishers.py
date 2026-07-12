from __future__ import annotations

import time

import numpy as np


def _low_latency_qos(queue_size: int = 1):
    """Build a sensor-data QoS profile: keep only the newest sample so a slow
    subscriber can never back-pressure the control loop.

    For live teleop telemetry (/joint_states, camera images) the newest value is
    the only one that matters, so depth-1 BEST_EFFORT drops stale samples
    instead of queuing them and avoids head-of-line blocking.
    """
    try:
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, int(queue_size)),
        )
    except ImportError:
        # rclpy not available (e.g. unit tests); fall back to a plain int depth.
        return max(1, int(queue_size))


class JointStatePublisher:
    def __init__(
        self,
        node,
        topic: str = "/joint_states",
        queue_size: int = 1,
        max_publish_hz: float | None = 30.0,
    ):
        from sensor_msgs.msg import JointState

        self._node = node
        self._joint_state_type = JointState
        self._publisher = node.create_publisher(JointState, topic, _low_latency_qos(queue_size))
        # /joint_states is telemetry: the robot is driven by apply_action, not by
        # this topic. Throttle it so publishing never competes with the control
        # loop, and cache the joint-name list so we don't rebuild it per frame.
        self._max_period_s = (
            None if max_publish_hz is None else 1.0 / max(1.0, float(max_publish_hz))
        )
        self._last_publish_s: float = 0.0
        self._cached_names: list[str] | None = None

    def publish(self, joint_names: list[str], joint_positions, stamp=None):
        # Rate-limit: skip publishing if we published too recently. The robot
        # motion itself is unaffected — it is driven by apply_action in the
        # adapter, not by this telemetry topic.
        now_s = time.monotonic()
        if self._max_period_s is not None and (now_s - self._last_publish_s) < self._max_period_s:
            return

        if stamp is None:
            stamp = self._node.get_clock().now().to_msg()

        positions = np.asarray(joint_positions, dtype=float).reshape(-1)
        message = self._joint_state_type()
        message.header.stamp = stamp
        # Reuse the name list across calls when it hasn't changed.
        if self._cached_names is None or len(self._cached_names) != len(joint_names):
            self._cached_names = list(joint_names)
        message.name = self._cached_names
        message.position = positions.tolist()
        self._publisher.publish(message)
        self._last_publish_s = now_s


class CameraImagePublishers:
    def __init__(self, node, camera_specs: dict, queue_size: int = 1):
        from sensor_msgs.msg import Image

        self._node = node
        self._image_type = Image
        self._publishers = {
            name: node.create_publisher(Image, spec.topic, _low_latency_qos(queue_size))
            for name, spec in camera_specs.items()
        }

    def has_subscribers(self, camera_name: str) -> bool:
        if camera_name not in self._publishers:
            return False
        try:
            return self._publishers[camera_name].get_subscription_count() > 0
        except Exception:
            # Fallback in case publisher API is mocked/unavailable
            return True

    def publish_rgb(self, camera_name: str, image_rgb, stamp=None, frame_id: str | None = None):
        if camera_name not in self._publishers:
            raise KeyError(f"Unknown camera publisher: {camera_name}")

        if stamp is None:
            stamp = self._node.get_clock().now().to_msg()

        image_array = np.asarray(image_rgb)
        if image_array.ndim != 3 or image_array.shape[2] not in (3, 4):
            raise ValueError(
                f"Expected RGB or RGBA image data for {camera_name}, got shape {image_array.shape}"
            )
        if image_array.shape[2] == 4:
            image_array = image_array[:, :, :3]

        image_array = np.asarray(image_array, dtype=np.uint8, order="C")
        message = self._image_type()
        message.header.stamp = stamp
        message.header.frame_id = frame_id or camera_name
        message.height = int(image_array.shape[0])
        message.width = int(image_array.shape[1])
        message.encoding = "rgb8"
        message.is_bigendian = False
        message.step = int(image_array.shape[1] * 3)
        message.data = image_array.tobytes()
        self._publishers[camera_name].publish(message)
