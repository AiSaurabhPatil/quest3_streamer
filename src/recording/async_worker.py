from __future__ import annotations

from dataclasses import dataclass
import queue
import threading

from .snapshots import RecordingFrameSnapshot


@dataclass
class RecorderDiagnostics:
    frames_enqueued: int = 0
    frames_written: int = 0
    frames_dropped_queue_full: int = 0
    save_requests: int = 0
    saved_episodes: int = 0
    discard_requests: int = 0
    discarded_episodes: int = 0
    empty_save_requests: int = 0
    worker_errors: int = 0
    queue_depth: int = 0
    last_error: str | None = None


@dataclass(frozen=True)
class RecorderCommand:
    kind: str
    reason: str | None = None


class RecorderWorker:
    def __init__(self, *, dataset, config, schema):
        self._dataset = dataset
        self._config = config
        self._schema = schema
        self._frame_queue: queue.Queue[RecordingFrameSnapshot] = queue.Queue(
            maxsize=config.queue_size_frames
        )
        self._command_queue: queue.Queue[RecorderCommand] = queue.Queue()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._buffered_frame_count = 0
        self.diagnostics = RecorderDiagnostics()

    def start(self):
        if self._thread is not None:
            return self
        self._thread = threading.Thread(
            target=self._run,
            name="lerobot-recorder",
            daemon=True,
        )
        self._thread.start()
        return self

    def enqueue_frame(self, snapshot: RecordingFrameSnapshot) -> bool:
        try:
            self._frame_queue.put_nowait(snapshot)
            self.diagnostics.frames_enqueued += 1
            self.diagnostics.queue_depth = self._frame_queue.qsize()
            return True
        except queue.Full:
            self.diagnostics.frames_dropped_queue_full += 1
            self.diagnostics.queue_depth = self._frame_queue.qsize()
            return False

    def request_save(self, reason: str | None = None) -> None:
        self.diagnostics.save_requests += 1
        self._command_queue.put(RecorderCommand(kind="save", reason=reason))

    def request_discard(self, reason: str | None = None) -> None:
        self.diagnostics.discard_requests += 1
        self._command_queue.put(RecorderCommand(kind="discard", reason=reason))

    def finalize(self, *, auto_save_pending: bool = False) -> None:
        self._command_queue.put(
            RecorderCommand(kind="finalize_save" if auto_save_pending else "finalize_discard")
        )
        if self._thread is not None:
            self._thread.join(timeout=30.0)
            self._thread = None

    def _run(self) -> None:
        while not self._stop_event.is_set():
            command = self._get_next_command()
            if command is not None:
                self._handle_command(command)
                continue

            try:
                snapshot = self._frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                self._dataset.add_frame(self._frame_to_payload(snapshot))
                self._buffered_frame_count += 1
                self.diagnostics.frames_written += 1
            except Exception as exc:
                self._record_error(exc)
            finally:
                self._frame_queue.task_done()
                self.diagnostics.queue_depth = self._frame_queue.qsize()

    def _get_next_command(self) -> RecorderCommand | None:
        try:
            return self._command_queue.get_nowait()
        except queue.Empty:
            return None

    def _handle_command(self, command: RecorderCommand) -> None:
        try:
            if command.kind == "save":
                self._drain_frame_queue()
                if self._buffered_frame_count > 0:
                    self._save_episode()
                else:
                    self.diagnostics.empty_save_requests += 1
                return
            if command.kind == "discard":
                self._clear_pending_frame_queue()
                self._dataset.clear_episode_buffer(delete_images=True)
                self._buffered_frame_count = 0
                self.diagnostics.discarded_episodes += 1
                return
            if command.kind == "finalize_save":
                self._drain_frame_queue()
                if self._buffered_frame_count > 0:
                    self._save_episode()
                self._dataset.finalize()
                self._stop_event.set()
                return
            if command.kind == "finalize_discard":
                self._clear_pending_frame_queue()
                if self._buffered_frame_count > 0:
                    self._dataset.clear_episode_buffer(delete_images=True)
                    self._buffered_frame_count = 0
                self._dataset.finalize()
                self._stop_event.set()
        except Exception as exc:
            self._record_error(exc)
            if command.kind.startswith("finalize"):
                self._stop_event.set()

    def _drain_frame_queue(self) -> None:
        while True:
            try:
                snapshot = self._frame_queue.get_nowait()
            except queue.Empty:
                self.diagnostics.queue_depth = 0
                return

            try:
                self._dataset.add_frame(self._frame_to_payload(snapshot))
                self._buffered_frame_count += 1
                self.diagnostics.frames_written += 1
            except Exception as exc:
                self._record_error(exc)
            finally:
                self._frame_queue.task_done()
                self.diagnostics.queue_depth = self._frame_queue.qsize()

    def _clear_pending_frame_queue(self) -> None:
        while True:
            try:
                _ = self._frame_queue.get_nowait()
            except queue.Empty:
                self.diagnostics.queue_depth = 0
                return
            self._frame_queue.task_done()
            self.diagnostics.queue_depth = self._frame_queue.qsize()

    def _save_episode(self) -> None:
        try:
            self._dataset.save_episode(parallel_encoding=self._config.save_parallel_encoding)
        except TypeError:
            self._dataset.save_episode()
        self._buffered_frame_count = 0
        self.diagnostics.saved_episodes += 1

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
