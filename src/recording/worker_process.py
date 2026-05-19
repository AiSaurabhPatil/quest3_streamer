from __future__ import annotations

import argparse
from dataclasses import asdict
import inspect
import json
from pathlib import Path
import shutil
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

        compat = LeRobotDatasetCompat.load(self._config.dataset_format)

        dataset_root = Path(self._config.root) / self._config.repo_id
        if dataset_root.exists():
            if _is_recreatable_empty_dataset_root(dataset_root, compat.dataset_format):
                shutil.rmtree(dataset_root)
                self._dataset = self._create_dataset(compat, dataset_root, robot_type)
                return
            _validate_existing_dataset_root(dataset_root, expected_format=compat.dataset_format)
            self._dataset = compat.open_dataset(
                repo_id=self._config.repo_id,
                root=dataset_root,
                streaming_encoding=self._config.streaming_encoding,
                vcodec=self._config.vcodec,
                encoder_threads=self._config.encoder_threads,
            )
            return

        self._dataset = self._create_dataset(compat, dataset_root, robot_type)

    def _create_dataset(self, compat, dataset_root: Path, robot_type: str):
        return compat.create_dataset(
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
        payload = self._frame_to_payload(snapshot)
        task = str(payload.pop("task", ""))
        try:
            self._dataset.add_frame(payload, task=task)
        except TypeError:
            payload["task"] = task
            self._dataset.add_frame(payload)
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
        try:
            self._dataset.clear_episode_buffer(delete_images=True)
        except TypeError:
            self._dataset.clear_episode_buffer()
        self._buffered_frame_count = 0
        self.diagnostics.discarded_episodes += 1

    def _finalize(self, *, auto_save_pending: bool) -> None:
        if auto_save_pending:
            self._save_episode()
        elif self._buffered_frame_count > 0:
            self._discard_episode()
        if hasattr(self._dataset, "finalize"):
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


class LeRobotDatasetCompat:
    def __init__(self, dataset_cls, *, codebase_version: str):
        self.dataset_cls = dataset_cls
        self.codebase_version = codebase_version

    @property
    def dataset_format(self) -> str:
        return "v2.1" if self.codebase_version.startswith("v2.") else "v3.0"

    @classmethod
    def load(cls, requested_format: str):
        try:
            try:
                import lerobot.common.datasets.lerobot_dataset as dataset_module
            except ImportError:
                import lerobot.datasets.lerobot_dataset as dataset_module
        except ImportError as exc:
            raise RuntimeError(
                "LeRobot is not installed in the recording worker Python environment "
                f"({sys.executable}). Install 'lerobot' there or set LEROBOT_RECORDING_PYTHON."
            ) from exc

        compat = cls(
            dataset_module.LeRobotDataset,
            codebase_version=str(getattr(dataset_module, "CODEBASE_VERSION", "unknown")),
        )
        if requested_format != "auto" and compat.dataset_format != requested_format:
            raise RuntimeError(
                "Recording dataset_format="
                f"{requested_format} requires a LeRobot worker that writes {requested_format}, "
                f"but {sys.executable} provides {compat.codebase_version}. "
                "Set LEROBOT_RECORDING_PYTHON to a matching environment."
            )
        return compat

    def open_dataset(self, **kwargs):
        return self.dataset_cls(**_supported_kwargs(self.dataset_cls, kwargs))

    def create_dataset(self, **kwargs):
        create = self.dataset_cls.create
        return create(**_supported_kwargs(create, kwargs))


def _supported_kwargs(callable_obj, kwargs: dict) -> dict:
    signature = inspect.signature(callable_obj)
    if any(param.kind == inspect.Parameter.VAR_KEYWORD for param in signature.parameters.values()):
        return kwargs
    return {key: value for key, value in kwargs.items() if key in signature.parameters}


def _validate_existing_dataset_root(dataset_root: Path, *, expected_format: str = "auto") -> None:
    dataset_format = _read_dataset_format(dataset_root)
    if expected_format != "auto" and dataset_format != "unknown" and dataset_format != expected_format:
        raise RuntimeError(
            f"Dataset path already exists with format {dataset_format}: {dataset_root}. "
            f"The selected LeRobot worker expects {expected_format}."
        )

    required_paths = _required_dataset_paths(dataset_root, dataset_format if dataset_format != "unknown" else expected_format)
    missing_paths = [path for path in required_paths if not path.exists()]
    if not missing_paths:
        return

    missing_text = ", ".join(str(path.relative_to(dataset_root)) for path in missing_paths)
    raise RuntimeError(
        f"Dataset path already exists but is incomplete: {dataset_root}. "
        f"Missing: {missing_text}. Move it aside, delete it, or use a new --dataset-repo-id."
    )


def _read_dataset_format(dataset_root: Path) -> str:
    info_path = dataset_root / "meta" / "info.json"
    if not info_path.exists():
        return "unknown"
    try:
        with info_path.open("r", encoding="utf-8") as info_file:
            version = str(json.load(info_file).get("codebase_version", "unknown"))
    except (OSError, json.JSONDecodeError):
        return "unknown"
    if version.startswith("v2."):
        return "v2.1"
    if version.startswith("v3."):
        return "v3.0"
    return "unknown"


def _is_recreatable_empty_dataset_root(dataset_root: Path, expected_format: str) -> bool:
    if expected_format != "v2.1" or _read_dataset_format(dataset_root) != "v2.1":
        return False
    try:
        info = json.loads((dataset_root / "meta" / "info.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    if int(info.get("total_episodes", 0)) != 0 or int(info.get("total_frames", 0)) != 0:
        return False
    episode_artifacts = list((dataset_root / "data").rglob("*.parquet")) + list(
        (dataset_root / "videos").rglob("*.mp4")
    )
    return not episode_artifacts


def _required_dataset_paths(dataset_root: Path, dataset_format: str) -> tuple[Path, ...]:
    if dataset_format == "v2.1":
        return (
            dataset_root / "meta" / "info.json",
            dataset_root / "meta" / "episodes.jsonl",
            dataset_root / "meta" / "episodes_stats.jsonl",
            dataset_root / "meta" / "tasks.jsonl",
            dataset_root / "data",
        )
    return (
        dataset_root / "meta" / "info.json",
        dataset_root / "meta" / "tasks.parquet",
        dataset_root / "meta" / "episodes",
        dataset_root / "data",
    )


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    return LeRobotWorkerProcess(read_fd=args.read_fd, write_fd=args.write_fd).run()


if __name__ == "__main__":
    raise SystemExit(main())
