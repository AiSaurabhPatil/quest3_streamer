from __future__ import annotations

from collections.abc import Iterable, Iterator
from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import Any

try:
    from src.teleop_core import (
        ControllerAxes,
        ControllerButtons,
        ControllerPose,
        ControllerState,
    )
except ModuleNotFoundError:
    from teleop_core import (
        ControllerAxes,
        ControllerButtons,
        ControllerPose,
        ControllerState,
    )


def controller_state_to_mapping(state: ControllerState | None) -> dict[str, Any] | None:
    if state is None:
        return None

    payload: dict[str, Any] = {
        "hand": str(state.hand),
        "sequence": int(state.sequence),
        "source_timestamp": float(state.source_timestamp),
        "axes": {
            "trigger": float(state.axes.trigger),
            "squeeze": float(state.axes.squeeze),
            "thumbstick_x": float(state.axes.thumbstick_x),
            "thumbstick_y": float(state.axes.thumbstick_y),
        },
        "buttons": {
            "primary": bool(state.buttons.primary),
            "secondary": bool(state.buttons.secondary),
            "menu": bool(state.buttons.menu),
            "stick_click": bool(state.buttons.stick_click),
        },
        "transport": {
            "client_epoch_ms": state.client_epoch_ms,
            "ingress_receive_epoch_ms": state.ingress_receive_epoch_ms,
            "remote_receive_epoch_ms": state.remote_receive_epoch_ms,
            "ros_publish_epoch_ms": state.ros_publish_epoch_ms,
            "control_receive_epoch_ms": state.control_receive_epoch_ms,
            "isaac_apply_epoch_ms": state.isaac_apply_epoch_ms,
        },
    }

    if state.pose is None:
        payload["pose"] = None
    else:
        payload["pose"] = {
            "position_xyz": state.pose.position_xyz.tolist(),
            "orientation_xyzw": state.pose.orientation_xyzw.tolist(),
            "valid": bool(state.pose.valid),
        }

    return payload


def controller_state_from_mapping(
    payload: dict[str, Any] | None,
    *,
    hand: str | None = None,
    receive_time_s: float = 0.0,
) -> ControllerState | None:
    if payload is None:
        return None

    state_hand = str(payload.get("hand", hand or "unknown"))
    axes_payload = payload.get("axes", {}) or {}
    buttons_payload = payload.get("buttons", {}) or {}
    transport_payload = payload.get("transport", {}) or {}
    pose_payload = payload.get("pose")

    pose = None
    if isinstance(pose_payload, dict):
        pose = ControllerPose(
            position_xyz=pose_payload.get("position_xyz", [0.0, 0.0, 0.0]),
            orientation_xyzw=pose_payload.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0]),
            valid=bool(pose_payload.get("valid", True)),
        )

    return ControllerState(
        hand=state_hand,
        sequence=int(payload.get("sequence", -1)),
        source_timestamp=float(payload.get("source_timestamp", receive_time_s)),
        receive_time_s=float(receive_time_s),
        pose=pose,
        axes=ControllerAxes(
            trigger=float(axes_payload.get("trigger", 0.0)),
            squeeze=float(axes_payload.get("squeeze", 0.0)),
            thumbstick_x=float(axes_payload.get("thumbstick_x", 0.0)),
            thumbstick_y=float(axes_payload.get("thumbstick_y", 0.0)),
        ),
        buttons=ControllerButtons(
            primary=bool(buttons_payload.get("primary", False)),
            secondary=bool(buttons_payload.get("secondary", False)),
            menu=bool(buttons_payload.get("menu", False)),
            stick_click=bool(buttons_payload.get("stick_click", False)),
        ),
        client_epoch_ms=_optional_float(transport_payload.get("client_epoch_ms")),
        ingress_receive_epoch_ms=_optional_float(
            transport_payload.get("ingress_receive_epoch_ms")
        ),
        remote_receive_epoch_ms=_optional_float(
            transport_payload.get("remote_receive_epoch_ms")
        ),
        ros_publish_epoch_ms=_optional_float(transport_payload.get("ros_publish_epoch_ms")),
        control_receive_epoch_ms=_optional_float(
            transport_payload.get("control_receive_epoch_ms")
        ),
        isaac_apply_epoch_ms=_optional_float(transport_payload.get("isaac_apply_epoch_ms")),
    )


def _optional_float(value) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


@dataclass
class ControllerStreamFrame:
    t: float
    left: ControllerState | None = None
    right: ControllerState | None = None
    extra: dict[str, Any] = field(default_factory=dict)

    def to_mapping(self) -> dict[str, Any]:
        payload = dict(self.extra)
        payload["t"] = float(self.t)
        payload["left"] = controller_state_to_mapping(self.left)
        payload["right"] = controller_state_to_mapping(self.right)
        return payload

    @classmethod
    def from_mapping(cls, payload: dict[str, Any]) -> "ControllerStreamFrame":
        frame_time_s = float(payload.get("t", 0.0))
        reserved = {"t", "left", "right"}
        return cls(
            t=frame_time_s,
            left=controller_state_from_mapping(
                payload.get("left"),
                hand="left",
                receive_time_s=frame_time_s,
            ),
            right=controller_state_from_mapping(
                payload.get("right"),
                hand="right",
                receive_time_s=frame_time_s,
            ),
            extra={key: value for key, value in payload.items() if key not in reserved},
        )

    def controller_states(self, receive_time_s: float | None = None) -> dict[str, ControllerState | None]:
        target_time_s = float(self.t if receive_time_s is None else receive_time_s)
        return {
            "left": controller_state_from_mapping(
                controller_state_to_mapping(self.left),
                hand="left",
                receive_time_s=target_time_s,
            ),
            "right": controller_state_from_mapping(
                controller_state_to_mapping(self.right),
                hand="right",
                receive_time_s=target_time_s,
            ),
        }


def iter_controller_stream(path: str | Path) -> Iterator[ControllerStreamFrame]:
    stream_path = Path(path)
    with stream_path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"Invalid JSONL in {stream_path} at line {line_number}: {exc}"
                ) from exc
            if not isinstance(payload, dict):
                raise ValueError(
                    f"Expected JSON object in {stream_path} at line {line_number}"
                )
            yield ControllerStreamFrame.from_mapping(payload)


def load_controller_stream(path: str | Path) -> list[ControllerStreamFrame]:
    return list(iter_controller_stream(path))


def write_controller_stream(
    path: str | Path,
    frames: Iterable[ControllerStreamFrame],
) -> Path:
    stream_path = Path(path)
    stream_path.parent.mkdir(parents=True, exist_ok=True)
    with stream_path.open("w", encoding="utf-8") as handle:
        for frame in frames:
            handle.write(json.dumps(frame.to_mapping(), separators=(",", ":")))
            handle.write("\n")
    return stream_path


def replay_session_updates(
    frames: Iterable[ControllerStreamFrame],
    session,
    *,
    start_time_s: float = 0.0,
):
    updates = []
    for frame in frames:
        now_s = float(start_time_s) + float(frame.t)
        updates.append(session.update(frame.controller_states(now_s), now_s=now_s))
    return updates
