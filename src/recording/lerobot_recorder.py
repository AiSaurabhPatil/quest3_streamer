from __future__ import annotations

from dataclasses import dataclass
import os

from .async_worker import RecorderWorker
from .snapshots import RecordingFrameSnapshot


@dataclass(frozen=True)
class RecorderStatus:
    enabled: bool
    repo_id: str
    root: str


class LeRobotEpisodeRecorder:
    def __init__(self, *, config, schema, robot_type: str):
        self._config = config
        self._schema = schema
        self._robot_type = robot_type
        self._dataset = self._create_dataset()
        self._worker = RecorderWorker(
            dataset=self._dataset,
            config=self._config,
            schema=self._schema,
        )

    @property
    def diagnostics(self):
        return self._worker.diagnostics

    @property
    def status(self) -> RecorderStatus:
        return RecorderStatus(
            enabled=True,
            repo_id=self._config.repo_id,
            root=self._config.root,
        )

    def start(self):
        self._worker.start()
        return self

    def enqueue_frame(self, snapshot: RecordingFrameSnapshot) -> bool:
        return self._worker.enqueue_frame(snapshot)

    def save_episode_async(self, *, reason: str | None = None) -> None:
        self._worker.request_save(reason=reason)

    def discard_episode_async(self, *, reason: str | None = None) -> None:
        self._worker.request_discard(reason=reason)

    def finalize(self) -> None:
        self._worker.finalize(auto_save_pending=self._config.auto_save_on_shutdown)
        if self._config.push_to_hub_on_shutdown:
            self._dataset.push_to_hub(private=self._config.private_hub_repo)

    def close(self) -> None:
        self.finalize()

    def _create_dataset(self):
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as exc:
            raise RuntimeError(
                "Recording is enabled but LeRobot is not installed. "
                "Install the 'lerobot' package in this environment to use dataset recording."
            ) from exc

        os.makedirs(self._config.root, exist_ok=True)
        return LeRobotDataset.create(
            repo_id=self._config.repo_id,
            root=self._config.root,
            fps=self._config.fps,
            features=self._schema.features,
            robot_type=self._robot_type,
            use_videos=self._config.use_videos,
            image_writer_threads=self._config.image_writer_threads,
            streaming_encoding=self._config.streaming_encoding,
            vcodec=self._config.vcodec,
            encoder_threads=self._config.encoder_threads,
        )
