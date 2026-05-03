from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any


def _optional_float(value) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _optional_int(value) -> int | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _format_optional_float(value) -> str | None:
    parsed = _optional_float(value)
    if parsed is None:
        return None
    return f"{parsed:.3f}"


def _parse_pose(mapping, key: str) -> tuple[float, ...] | None:
    if not isinstance(mapping, Mapping):
        return None
    vector = mapping.get(key)
    if not isinstance(vector, Mapping):
        return None

    if key == "position":
        coords = ("x", "y", "z")
    else:
        coords = ("x", "y", "z", "w")

    values: list[float] = []
    for coord in coords:
        value = _optional_float(vector.get(coord))
        if value is None:
            return None
        values.append(value)
    return tuple(values)


@dataclass
class TransportTimestamps:
    client_epoch_ms: float | None = None
    ingress_receive_epoch_ms: float | None = None
    remote_receive_epoch_ms: float | None = None
    control_consume_epoch_ms: float | None = None
    isaac_apply_epoch_ms: float | None = None

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "TransportTimestamps":
        transport = data.get("transport", {})
        if not isinstance(transport, Mapping):
            transport = {}

        return cls(
            client_epoch_ms=_optional_float(
                transport.get("client_epoch_ms", data.get("client_epoch_ms"))
            ),
            ingress_receive_epoch_ms=_optional_float(transport.get("ingress_receive_epoch_ms")),
            remote_receive_epoch_ms=_optional_float(transport.get("remote_receive_epoch_ms")),
            control_consume_epoch_ms=_optional_float(transport.get("control_consume_epoch_ms")),
            isaac_apply_epoch_ms=_optional_float(transport.get("isaac_apply_epoch_ms")),
        )

    def to_mapping(self) -> dict[str, float]:
        mapping: dict[str, float] = {}
        if self.client_epoch_ms is not None:
            mapping["client_epoch_ms"] = float(self.client_epoch_ms)
        if self.ingress_receive_epoch_ms is not None:
            mapping["ingress_receive_epoch_ms"] = float(self.ingress_receive_epoch_ms)
        if self.remote_receive_epoch_ms is not None:
            mapping["remote_receive_epoch_ms"] = float(self.remote_receive_epoch_ms)
        if self.control_consume_epoch_ms is not None:
            mapping["control_consume_epoch_ms"] = float(self.control_consume_epoch_ms)
        if self.isaac_apply_epoch_ms is not None:
            mapping["isaac_apply_epoch_ms"] = float(self.isaac_apply_epoch_ms)
        return mapping


@dataclass
class ControllerPacketState:
    position_xyz: tuple[float, float, float] | None = None
    orientation_xyzw: tuple[float, float, float, float] | None = None
    trigger: float = 0.0
    squeeze: float = 0.0
    thumbstick_x: float = 0.0
    thumbstick_y: float = 0.0
    button_a_x: bool = False
    button_b_y: bool = False
    thumbstick_click: bool = False
    extra: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "ControllerPacketState":
        reserved = {
            "position",
            "orientation",
            "trigger",
            "squeeze",
            "thumbstick_x",
            "thumbstick_y",
            "button_a_x",
            "button_b_y",
            "thumbstick_click",
        }
        return cls(
            position_xyz=_parse_pose(data, "position"),
            orientation_xyzw=_parse_pose(data, "orientation"),
            trigger=float(_optional_float(data.get("trigger")) or 0.0),
            squeeze=float(_optional_float(data.get("squeeze")) or 0.0),
            thumbstick_x=float(_optional_float(data.get("thumbstick_x")) or 0.0),
            thumbstick_y=float(_optional_float(data.get("thumbstick_y")) or 0.0),
            button_a_x=bool(data.get("button_a_x", False)),
            button_b_y=bool(data.get("button_b_y", False)),
            thumbstick_click=bool(data.get("thumbstick_click", False)),
            extra={key: value for key, value in data.items() if key not in reserved},
        )

    def to_mapping(self) -> dict[str, Any]:
        mapping = dict(self.extra)
        if self.position_xyz is not None:
            mapping["position"] = {
                "x": float(self.position_xyz[0]),
                "y": float(self.position_xyz[1]),
                "z": float(self.position_xyz[2]),
            }
        else:
            mapping["position"] = None

        if self.orientation_xyzw is not None:
            mapping["orientation"] = {
                "x": float(self.orientation_xyzw[0]),
                "y": float(self.orientation_xyzw[1]),
                "z": float(self.orientation_xyzw[2]),
                "w": float(self.orientation_xyzw[3]),
            }
        else:
            mapping["orientation"] = None

        mapping["trigger"] = float(self.trigger)
        mapping["squeeze"] = float(self.squeeze)
        mapping["thumbstick_x"] = float(self.thumbstick_x)
        mapping["thumbstick_y"] = float(self.thumbstick_y)
        mapping["button_a_x"] = bool(self.button_a_x)
        mapping["button_b_y"] = bool(self.button_b_y)
        mapping["thumbstick_click"] = bool(self.thumbstick_click)
        return mapping


@dataclass
class QuestPacket:
    schema_version: int = 1
    sequence: int | None = None
    timestamp: float | None = None
    transport: TransportTimestamps = field(default_factory=TransportTimestamps)
    controllers: dict[str, ControllerPacketState] = field(default_factory=dict)
    extra: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "QuestPacket":
        if not isinstance(data, Mapping):
            raise ValueError("Expected Quest packet to be a JSON object")

        reserved = {
            "schema_version",
            "sequence",
            "timestamp",
            "client_epoch_ms",
            "transport",
            "controllers",
        }
        controllers_payload = data.get("controllers", {})
        controllers: dict[str, ControllerPacketState] = {}
        if isinstance(controllers_payload, Mapping):
            for hand, controller_payload in controllers_payload.items():
                if isinstance(controller_payload, Mapping):
                    controllers[str(hand)] = ControllerPacketState.from_mapping(controller_payload)

        return cls(
            schema_version=int(_optional_int(data.get("schema_version")) or 1),
            sequence=_optional_int(data.get("sequence")),
            timestamp=_optional_float(data.get("timestamp")),
            transport=TransportTimestamps.from_mapping(data),
            controllers=controllers,
            extra={key: value for key, value in data.items() if key not in reserved},
        )

    def to_mapping(self) -> dict[str, Any]:
        payload = dict(self.extra)
        payload["schema_version"] = int(self.schema_version)
        payload["sequence"] = self.sequence
        payload["timestamp"] = self.timestamp
        payload["client_epoch_ms"] = self.transport.client_epoch_ms
        payload["transport"] = self.transport.to_mapping()
        payload["controllers"] = {
            hand: controller.to_mapping() for hand, controller in self.controllers.items()
        }
        return payload


def format_ros_frame_id(
    packet: QuestPacket,
    *,
    base_frame: str = "quest_world",
    ros_publish_epoch_ms: float | None = None,
) -> str:
    """Encode packet metadata into a ROS Header frame_id without changing topic types."""
    fields: list[tuple[str, str]] = []
    if packet.sequence is not None:
        fields.append(("seq", str(int(packet.sequence))))

    timestamp = _format_optional_float(packet.timestamp)
    if timestamp is not None:
        fields.append(("ts", timestamp))

    transport_fields = (
        ("client", packet.transport.client_epoch_ms),
        ("ingress", packet.transport.ingress_receive_epoch_ms),
        ("remote", packet.transport.remote_receive_epoch_ms),
        ("ros", ros_publish_epoch_ms),
        ("control", packet.transport.control_consume_epoch_ms),
        ("isaac", packet.transport.isaac_apply_epoch_ms),
    )
    for key, value in transport_fields:
        formatted = _format_optional_float(value)
        if formatted is not None:
            fields.append((key, formatted))

    if not fields:
        return base_frame
    return "|".join([base_frame, *(f"{key}={value}" for key, value in fields)])


def parse_ros_frame_id_metadata(frame_id: str) -> tuple[str, dict[str, float | int]]:
    """Decode metadata produced by format_ros_frame_id.

    Unknown fields are ignored so older/newer bridges remain compatible.
    """
    if not frame_id:
        return "", {}

    parts = str(frame_id).split("|")
    metadata: dict[str, float | int] = {}
    for part in parts[1:]:
        if "=" not in part:
            continue
        key, raw_value = part.split("=", 1)
        if key == "seq":
            parsed_int = _optional_int(raw_value)
            if parsed_int is not None:
                metadata[key] = parsed_int
            continue
        parsed_float = _optional_float(raw_value)
        if parsed_float is not None:
            metadata[key] = parsed_float
    return parts[0], metadata
