from __future__ import annotations

import argparse
from dataclasses import asdict
from pathlib import Path
import sys
import traceback

from .async_worker import RecorderDiagnostics
from .ipc import IPCClosedError, receive_message, send_message
from .snapshots import RecordingFrameSnapshot


class LeRobotWorkerProcess:
    def __init__(self, *, read_fd: int, write_fd: int):
        self._read_fd = read_fd
        self._write_fd = write_fd
        self._dataset = None
        self._config = None
        self._schema = None
        self._buffered_frame_count = 0
        self.diagnostics = RecorderDiagnostics()

    def run(self) -> int:
        try:
            init_message = receive_message(self._read_fd)
            if init_message.get("type") != "init":
                raise ValueError("First recorder worker message must be 'init'")
            self._initialize(init_message)
            self._send_status("ready")
        except Exception as exc:
            self._send_error("init_error", exc)
            return 1

        while True:
            try:
                message = receive_message(self._read_fd)
            except IPCClosedError:
                return 0
            except Exception as exc:
                self._record_error(exc)
                self._send_status("error")
                continue

            try:
                should_exit = self._handle_message(message)
            except Exception as exc:
                self._record_error(exc)
                self._send_status("error")
                should_exit = False

            if should_exit:
                return 0

    def _initialize(self, message: dict[str, object]) -> None:
        self._config = message["config"]
        self._schema = message["schema"]
        robot_type = str(message["robot_type"])

        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as exc:
            raise RuntimeError(
                "LeRobot is not installed in the recording worker Python environment "
                f"({sys.executable}). Install 'lerobot' there or set LEROBOT_RECORDING_PYTHON."
            ) from exc

        dataset_root = Path(self._config.root) / self._config.repo_id
        if dataset_root.exists():
            _validate_existing_dataset_root(dataset_root)
            self._dataset = LeRobotDataset(
                repo_id=self._config.repo_id,
                root=dataset_root,
                streaming_encoding=self._config.streaming_encoding,
                vcodec=self._config.vcodec,
                encoder_threads=self._config.encoder_threads,
            )
            return

        self._dataset = self._create_dataset(LeRobotDataset, dataset_root, robot_type)

    def _create_dataset(self, LeRobotDataset, dataset_root: Path, robot_type: str):
        return LeRobotDataset.create(
            repo_id=self._config.repo_id,
            root=dataset_root,
            fps=self._config.fps,
            features=self._schema.features,
            robot_type=robot_type,
            use_videos=self._config.use_videos,
            image_writer_threads=self._config.image_writer_threads,
            streaming_encoding=self._config.streaming_encoding,
            vcodec=self._config.vcodec,
            encoder_threads=self._config.encoder_threads,
        )

    def _handle_message(self, message: dict[str, object]) -> bool:
        message_type = message.get("type")
        if message_type == "frame":
            self._add_frame(message["snapshot"])
            return False
        if message_type == "save":
            self.diagnostics.save_requests += 1
            self._save_episode()
            self._send_status("saved")
            return False
        if message_type == "discard":
            self.diagnostics.discard_requests += 1
            self._discard_episode()
            self._send_status("discarded")
            return False
        if message_type == "finalize":
            self._finalize(auto_save_pending=bool(message.get("auto_save_pending", False)))
            self._send_status("finalized")
            return True
        raise ValueError(f"Unknown recorder worker message type: {message_type}")

    def _add_frame(self, snapshot: RecordingFrameSnapshot) -> None:
        self._dataset.add_frame(self._frame_to_payload(snapshot))
        self._buffered_frame_count += 1
        self.diagnostics.frames_enqueued += 1
        self.diagnostics.frames_written += 1

    def _save_episode(self) -> None:
        if self._buffered_frame_count <= 0:
            self.diagnostics.empty_save_requests += 1
            return
        try:
            self._dataset.save_episode(parallel_encoding=self._config.save_parallel_encoding)
        except TypeError:
            self._dataset.save_episode()
        self._buffered_frame_count = 0
        self.diagnostics.saved_episodes += 1

    def _discard_episode(self) -> None:
        self._dataset.clear_episode_buffer(delete_images=True)
        self._buffered_frame_count = 0
        self.diagnostics.discarded_episodes += 1

    def _finalize(self, *, auto_save_pending: bool) -> None:
        if auto_save_pending:
            self._save_episode()
        elif self._buffered_frame_count > 0:
            self._discard_episode()
        self._dataset.finalize()
        if self._config.push_to_hub_on_shutdown:
            self._dataset.push_to_hub(private=self._config.private_hub_repo)

    def _frame_to_payload(self, snapshot: RecordingFrameSnapshot) -> dict[str, object]:
        payload: dict[str, object] = {
            self._schema.state_spec.key: snapshot.state,
            self._schema.action_spec.key: snapshot.action,
            "task": snapshot.task,
        }
        for camera_spec in self._schema.camera_specs:
            payload[camera_spec.feature_key] = snapshot.cameras[camera_spec.camera_name]
        return payload

    def _record_error(self, exc: Exception) -> None:
        self.diagnostics.worker_errors += 1
        self.diagnostics.last_error = f"{type(exc).__name__}: {exc}"

    def _send_status(self, message_type: str) -> None:
        send_message(
            self._write_fd,
            {
                "type": message_type,
                "diagnostics": asdict(self.diagnostics),
            },
        )

    def _send_error(self, message_type: str, exc: Exception) -> None:
        send_message(
            self._write_fd,
            {
                "type": message_type,
                "error": f"{type(exc).__name__}: {exc}",
                "traceback": traceback.format_exc(),
                "diagnostics": asdict(self.diagnostics),
            },
        )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="LeRobot dataset recording worker")
    parser.add_argument("--read-fd", required=True, type=int, help="File descriptor to read parent messages from")
    parser.add_argument("--write-fd", required=True, type=int, help="File descriptor to write parent messages to")
    return parser


def _validate_existing_dataset_root(dataset_root: Path) -> None:
    required_paths = (
        dataset_root / "meta" / "info.json",
        dataset_root / "meta" / "tasks.parquet",
        dataset_root / "meta" / "episodes",
        dataset_root / "data",
    )
    missing_paths = [path for path in required_paths if not path.exists()]
    if not missing_paths:
        return

    missing_text = ", ".join(str(path.relative_to(dataset_root)) for path in missing_paths)
    raise RuntimeError(
        f"Dataset path already exists but is incomplete: {dataset_root}. "
        f"Missing: {missing_text}. Move it aside, delete it, or use a new --dataset-repo-id."
    )


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    return LeRobotWorkerProcess(read_fd=args.read_fd, write_fd=args.write_fd).run()


if __name__ == "__main__":
    raise SystemExit(main())
