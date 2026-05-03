from __future__ import annotations

from dataclasses import dataclass, field
import os
from typing import Any


VALID_BUTTON_SYMBOLS = {
    "right_primary",
    "right_secondary",
    "left_primary",
    "left_secondary",
    "any_primary",
    "any_secondary",
}


@dataclass(frozen=True)
class RecordingButtonConfig:
    switch_camera: str = "right_primary"
    reset_scene: str = "right_secondary"
    save_episode: str = "left_primary"
    discard_episode: str = "left_secondary"

    @classmethod
    def from_mapping(cls, values: dict[str, Any] | None):
        values = values or {}
        config = cls(
            switch_camera=str(values.get("switch_camera", "right_primary")),
            reset_scene=str(values.get("reset_scene", "right_secondary")),
            save_episode=str(values.get("save_episode", "left_primary")),
            discard_episode=str(values.get("discard_episode", "left_secondary")),
        )
        config.validate()
        return config

    def validate(self) -> None:
        for value in (
            self.switch_camera,
            self.reset_scene,
            self.save_episode,
            self.discard_episode,
        ):
            if value not in VALID_BUTTON_SYMBOLS:
                raise ValueError(f"Unsupported recording button symbol '{value}'")


@dataclass(frozen=True)
class RecordingResetPolicy:
    discard_unsaved_episode: bool = True

    @classmethod
    def from_mapping(cls, values: dict[str, Any] | None):
        values = values or {}
        return cls(
            discard_unsaved_episode=bool(values.get("discard_unsaved_episode", True))
        )


@dataclass(frozen=True)
class RecordingVectorConfig:
    key: str
    dtype: str = "float32"
    mode: str = "named_groups"
    groups: tuple[str, ...] = ()

    @classmethod
    def from_mapping(cls, values: dict[str, Any] | None, *, default_key: str):
        values = values or {}
        groups = tuple(str(group) for group in values.get("groups", ()))
        return cls(
            key=str(values.get("key", default_key)),
            dtype=str(values.get("dtype", "float32")),
            mode=str(values.get("mode", "named_groups")),
            groups=groups,
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "dtype": self.dtype,
            "mode": self.mode,
            "groups": list(self.groups),
        }


@dataclass(frozen=True)
class RecordingCameraConfig:
    enabled: bool = True
    include: tuple[str, ...] = ()
    feature_prefix: str = "observation.images"
    dtype: str = "video"
    resolution: tuple[int, int] = (480, 360)

    @classmethod
    def from_mapping(cls, values: dict[str, Any] | None):
        values = values or {}
        resolution = values.get("resolution", (480, 360))
        if len(resolution) != 2:
            raise ValueError("Recording camera resolution must contain [width, height]")
        width = int(resolution[0])
        height = int(resolution[1])
        if width <= 0 or height <= 0:
            raise ValueError("Recording camera resolution must be positive")
        return cls(
            enabled=bool(values.get("enabled", True)),
            include=tuple(str(name) for name in values.get("include", ())),
            feature_prefix=str(values.get("feature_prefix", "observation.images")),
            dtype=str(values.get("dtype", "video")),
            resolution=(width, height),
        )


@dataclass(frozen=True)
class RecordingConfig:
    enabled: bool = False
    root: str = "datasets"
    repo_id: str = "local/quest3-openarm"
    task: str = "Teleoperate OpenArm to complete the task"
    fps: int = 30
    start_after_calibration: bool = True
    auto_start_episode: bool = True
    use_videos: bool = True
    streaming_encoding: bool = True
    vcodec: str = "auto"
    encoder_threads: int = 2
    image_writer_threads: int = 2
    queue_size_frames: int = 8
    drop_when_full: bool = True
    save_parallel_encoding: bool = True
    finalize_on_shutdown: bool = True
    push_to_hub_on_shutdown: bool = False
    private_hub_repo: bool = False
    auto_save_on_shutdown: bool = False
    buttons: RecordingButtonConfig = field(default_factory=RecordingButtonConfig)
    reset_policy: RecordingResetPolicy = field(default_factory=RecordingResetPolicy)
    state: RecordingVectorConfig = field(
        default_factory=lambda: RecordingVectorConfig(key="observation.state")
    )
    action: RecordingVectorConfig = field(
        default_factory=lambda: RecordingVectorConfig(key="action")
    )
    cameras: RecordingCameraConfig = field(default_factory=RecordingCameraConfig)

    @classmethod
    def from_mapping(cls, values: dict[str, Any] | None, *, project_root: str):
        values = values or {}
        root = str(values.get("root", "datasets"))
        if not os.path.isabs(root):
            root = os.path.join(project_root, root)
        config = cls(
            enabled=bool(values.get("enabled", False)),
            root=root,
            repo_id=str(values.get("repo_id", "local/quest3-openarm")),
            task=str(values.get("task", "Teleoperate OpenArm to complete the task")),
            fps=int(values.get("fps", 30)),
            start_after_calibration=bool(values.get("start_after_calibration", True)),
            auto_start_episode=bool(values.get("auto_start_episode", True)),
            use_videos=bool(values.get("use_videos", True)),
            streaming_encoding=bool(values.get("streaming_encoding", True)),
            vcodec=str(values.get("vcodec", "auto")),
            encoder_threads=int(values.get("encoder_threads", 2)),
            image_writer_threads=int(values.get("image_writer_threads", 2)),
            queue_size_frames=int(values.get("queue_size_frames", 8)),
            drop_when_full=bool(values.get("drop_when_full", True)),
            save_parallel_encoding=bool(values.get("save_parallel_encoding", True)),
            finalize_on_shutdown=bool(values.get("finalize_on_shutdown", True)),
            push_to_hub_on_shutdown=bool(values.get("push_to_hub_on_shutdown", False)),
            private_hub_repo=bool(values.get("private_hub_repo", False)),
            auto_save_on_shutdown=bool(values.get("auto_save_on_shutdown", False)),
            buttons=RecordingButtonConfig.from_mapping(values.get("buttons")),
            reset_policy=RecordingResetPolicy.from_mapping(values.get("reset_policy")),
            state=RecordingVectorConfig.from_mapping(
                values.get("state"),
                default_key="observation.state",
            ),
            action=RecordingVectorConfig.from_mapping(
                values.get("action"),
                default_key="action",
            ),
            cameras=RecordingCameraConfig.from_mapping(values.get("cameras")),
        )
        config.validate()
        return config

    def validate(self) -> None:
        if self.fps <= 0:
            raise ValueError("Recording fps must be positive")
        if self.queue_size_frames < 1:
            raise ValueError("Recording queue_size_frames must be at least 1")
        if self.encoder_threads < 1:
            raise ValueError("Recording encoder_threads must be at least 1")
        if self.image_writer_threads < 1:
            raise ValueError("Recording image_writer_threads must be at least 1")
