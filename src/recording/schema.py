from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .config import RecordingCameraConfig, RecordingConfig, RecordingVectorConfig


@dataclass(frozen=True)
class VectorSpec:
    key: str
    names: tuple[str, ...]
    shape: tuple[int, ...]
    dtype: str


@dataclass(frozen=True)
class CameraFeatureSpec:
    camera_name: str
    feature_key: str
    shape: tuple[int, int, int]
    dtype: str = "video"


@dataclass(frozen=True)
class RecordingSchema:
    features: dict[str, dict[str, Any]]
    state_spec: VectorSpec
    action_spec: VectorSpec
    camera_specs: tuple[CameraFeatureSpec, ...]
    auxiliary_specs: tuple[VectorSpec, ...] = ()


def build_recording_schema(adapter, robot_config: dict, recording_config: RecordingConfig) -> RecordingSchema:
    state_names = _resolve_vector_names(robot_config, recording_config.state, adapter.get_joint_names())
    action_names = _resolve_vector_names(robot_config, recording_config.action, adapter.get_joint_names())
    camera_specs = _resolve_camera_specs(adapter.get_camera_specs(), recording_config.cameras)

    features: dict[str, dict[str, Any]] = {
        recording_config.state.key: {
            "dtype": recording_config.state.dtype,
            "shape": (len(state_names),),
            "names": list(state_names),
        },
        recording_config.action.key: {
            "dtype": recording_config.action.dtype,
            "shape": (len(action_names),),
            "names": list(action_names),
        },
    }
    auxiliary_specs: list[VectorSpec] = []
    if recording_config.intervention.enabled:
        for key in (
            "action.human_valid",
            "action.policy_valid",
            "control.source",
            "intervention.active",
            "intervention.left",
            "intervention.right",
            "intervention.id",
            "intervention.reentry_blend_active",
        ):
            spec = VectorSpec(key=key, names=(key,), shape=(1,), dtype="int64")
            auxiliary_specs.append(spec)
            features[key] = {"dtype": spec.dtype, "shape": spec.shape, "names": list(spec.names)}
        if recording_config.intervention.include_candidate_actions:
            for key in ("action.human", "action.policy"):
                spec = VectorSpec(
                    key=key,
                    names=action_names,
                    shape=(len(action_names),),
                    dtype=recording_config.action.dtype,
                )
                auxiliary_specs.append(spec)
                features[key] = {
                    "dtype": spec.dtype,
                    "shape": spec.shape,
                    "names": list(spec.names),
                }
    for camera_spec in camera_specs:
        features[camera_spec.feature_key] = {
            "dtype": camera_spec.dtype,
            "shape": camera_spec.shape,
            "names": ["channel", "height", "width"],
        }

    return RecordingSchema(
        features=features,
        state_spec=VectorSpec(
            key=recording_config.state.key,
            names=state_names,
            shape=(len(state_names),),
            dtype=recording_config.state.dtype,
        ),
        action_spec=VectorSpec(
            key=recording_config.action.key,
            names=action_names,
            shape=(len(action_names),),
            dtype=recording_config.action.dtype,
        ),
        camera_specs=camera_specs,
        auxiliary_specs=tuple(auxiliary_specs),
    )


def _resolve_vector_names(
    robot_config: dict,
    vector_config: RecordingVectorConfig,
    articulation_joint_names: list[str],
) -> tuple[str, ...]:
    if vector_config.mode == "articulation_joints":
        return tuple(articulation_joint_names)

    if vector_config.mode != "named_groups":
        raise ValueError(f"Unsupported recording vector mode '{vector_config.mode}'")

    joint_groups = robot_config.get("recording", {}).get("joint_groups", {})
    names: list[str] = []
    for group_name in vector_config.groups:
        group = joint_groups.get(group_name)
        if group is None:
            raise KeyError(f"Unknown recording joint group '{group_name}'")
        source = str(group.get("source", "articulation"))
        if source == "articulation":
            names.extend(str(name) for name in group.get("joints", ()))
        elif source == "adapter":
            names.append(str(group.get("name", group_name)))
        else:
            raise ValueError(f"Unsupported recording group source '{source}'")
    return tuple(names)


def _resolve_camera_specs(camera_specs: dict, config: RecordingCameraConfig) -> tuple[CameraFeatureSpec, ...]:
    if not config.enabled:
        return ()

    width, height = config.resolution
    include = config.include or tuple(camera_specs.keys())
    resolved: list[CameraFeatureSpec] = []
    for camera_name in include:
        if camera_name not in camera_specs:
            raise KeyError(f"Unknown recording camera '{camera_name}'")
        resolved.append(
            CameraFeatureSpec(
                camera_name=camera_name,
                feature_key=f"{config.feature_prefix}.{camera_name}",
                shape=(3, height, width),
                dtype=config.dtype,
            )
        )
    return tuple(resolved)
