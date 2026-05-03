from __future__ import annotations

import numpy as np


class JointStatePublisher:
    def __init__(self, node, topic: str = "/joint_states", queue_size: int = 10):
        from sensor_msgs.msg import JointState

        self._node = node
        self._joint_state_type = JointState
        self._publisher = node.create_publisher(JointState, topic, queue_size)

    def publish(self, joint_names: list[str], joint_positions, stamp=None):
        if stamp is None:
            stamp = self._node.get_clock().now().to_msg()

        positions = np.asarray(joint_positions, dtype=float).reshape(-1)
        message = self._joint_state_type()
        message.header.stamp = stamp
        message.name = list(joint_names)
        message.position = positions.tolist()
        self._publisher.publish(message)


class CameraImagePublishers:
    def __init__(self, node, camera_specs: dict, queue_size: int = 10):
        from sensor_msgs.msg import Image

        self._node = node
        self._image_type = Image
        self._publishers = {
            name: node.create_publisher(Image, spec.topic, queue_size)
            for name, spec in camera_specs.items()
        }

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
