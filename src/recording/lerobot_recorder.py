from __future__ import annotations

from dataclasses import dataclass
import os
import queue
import subprocess
import threading
import time
from typing import Any

from .async_worker import RecorderDiagnostics
from .ipc import IPCClosedError, receive_message, send_message
from .snapshots import RecordingFrameSnapshot


WORKER_SHUTDOWN_TIMEOUT_S = 30.0


@dataclass(frozen=True)
class RecorderStatus:
    enabled: bool
    repo_id: str
    root: str


@dataclass(frozen=True)
class RecorderWorkerProcessStatus:
    python: str
    pid: int | None


class LeRobotEpisodeRecorder:
    def __init__(self, *, config, schema, robot_type: str, project_root: str | None = None):
        self._config = config
        self._schema = schema
        self._robot_type = robot_type
        self._project_root = project_root or os.getcwd()
        self._recording_python = self._resolve_recording_python()
        self._diagnostics = RecorderDiagnostics()
        self._send_queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=config.queue_size_frames)
        self._send_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._worker_read_fd: int | None = None
        self._worker_write_fd: int | None = None
        self._worker_process: subprocess.Popen | None = None
        self._worker_stderr_thread: threading.Thread | None = None
        self._reader_thread: threading.Thread | None = None
        self._writer_thread: threading.Thread | None = None

    @property
    def diagnostics(self):
        return self._diagnostics

    @property
    def status(self) -> RecorderStatus:
        return RecorderStatus(
            enabled=True,
            repo_id=self._config.repo_id,
            root=self._config.root,
        )

    @property
    def worker_process(self) -> RecorderWorkerProcessStatus:
        return RecorderWorkerProcessStatus(
            python=self._recording_python,
            pid=None if self._worker_process is None else self._worker_process.pid,
        )

    def start(self):
        if self._worker_process is not None:
            return self
        self._start_worker_process()
        return self

    def enqueue_frame(self, snapshot: RecordingFrameSnapshot) -> bool:
        if self._stop_event.is_set():
            self._diagnostics.frames_dropped_queue_full += 1
            return False
        try:
            self._send_queue.put_nowait({"type": "frame", "snapshot": snapshot})
            self._diagnostics.frames_enqueued += 1
            self._diagnostics.queue_depth = self._send_queue.qsize()
            return True
        except queue.Full:
            self._diagnostics.frames_dropped_queue_full += 1
            self._diagnostics.queue_depth = self._send_queue.qsize()
            return False

    def save_episode_async(self, *, reason: str | None = None) -> None:
        self._diagnostics.save_requests += 1
        self._enqueue_command({"type": "save", "reason": reason})

    def discard_episode_async(self, *, reason: str | None = None) -> None:
        self._diagnostics.discard_requests += 1
        self._enqueue_command({"type": "discard", "reason": reason})

    def finalize(self) -> None:
        if self._worker_process is None:
            return
        self._enqueue_command(
            {
                "type": "finalize",
                "auto_save_pending": self._config.auto_save_on_shutdown,
            }
        )
        self._wait_for_send_queue()
        self._wait_for_worker_shutdown()
        self._stop_event.set()
        self._close_pipes()

    def close(self) -> None:
        self.finalize()

    def _resolve_recording_python(self) -> str:
        configured_python = os.environ.get("LEROBOT_RECORDING_PYTHON")
        if configured_python:
            return configured_python
        if self._config.dataset_format == "v2.1":
            v21_python = os.path.join(self._project_root, ".venv-lerobot-v21", "bin", "python")
            if os.path.exists(v21_python):
                return v21_python
        return os.path.join(self._project_root, ".venv", "bin", "python")

    def _start_worker_process(self) -> None:
        if not os.path.exists(self._recording_python):
            raise RuntimeError(
                "Recording is enabled but the LeRobot worker Python was not found at "
                f"{self._recording_python}. Create the project .venv or set LEROBOT_RECORDING_PYTHON."
            )

        parent_read_fd, child_write_fd = os.pipe()
        child_read_fd, parent_write_fd = os.pipe()
        self._worker_read_fd = parent_read_fd
        self._worker_write_fd = parent_write_fd

        env = _build_worker_env(self._project_root)
        self._worker_process = subprocess.Popen(
            [
                self._recording_python,
                "-m",
                "src.recording.worker_process",
                "--read-fd",
                str(child_read_fd),
                "--write-fd",
                str(child_write_fd),
            ],
            cwd=self._project_root,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            pass_fds=(child_read_fd, child_write_fd),
        )
        os.close(child_read_fd)
        os.close(child_write_fd)
        self._worker_stderr_thread = threading.Thread(
            target=self._drain_worker_stderr,
            name="lerobot-recorder-stderr",
            daemon=True,
        )
        self._worker_stderr_thread.start()

        try:
            self._send_message(
                {
                    "type": "init",
                    "config": self._config,
                    "schema": self._schema,
                    "robot_type": self._robot_type,
                }
            )
            init_response = receive_message(self._worker_read_fd)
            if init_response.get("type") != "ready":
                error = init_response.get("error", "unknown worker initialization error")
                traceback_text = init_response.get("traceback", "")
                raise RuntimeError(f"{error}\n{traceback_text}".rstrip())
            self._apply_worker_diagnostics(init_response.get("diagnostics", {}))
        except Exception:
            self._stop_worker_after_failed_start()
            raise

        self._reader_thread = threading.Thread(
            target=self._read_worker_messages,
            name="lerobot-recorder-reader",
            daemon=True,
        )
        self._writer_thread = threading.Thread(
            target=self._write_worker_messages,
            name="lerobot-recorder-writer",
            daemon=True,
        )
        self._reader_thread.start()
        self._writer_thread.start()
        print(
            "[Recording] LeRobot writer process started "
            f"with {self._recording_python} (pid={self._worker_process.pid})"
        )

    def _enqueue_command(self, message: dict[str, Any]) -> None:
        try:
            self._send_queue.put_nowait(message)
            self._diagnostics.queue_depth = self._send_queue.qsize()
            return
        except queue.Full:
            self._drop_queued_frame_for_command()

        try:
            self._send_queue.put_nowait(message)
            self._diagnostics.queue_depth = self._send_queue.qsize()
        except queue.Full:
            self._diagnostics.worker_errors += 1
            self._diagnostics.last_error = f"Recorder command queue is full; dropped {message.get('type')} command"

    def _drop_queued_frame_for_command(self) -> None:
        deferred_messages = []
        dropped_frame = False
        while not dropped_frame:
            try:
                queued_message = self._send_queue.get_nowait()
            except queue.Empty:
                break
            try:
                if queued_message.get("type") == "frame":
                    self._diagnostics.frames_dropped_queue_full += 1
                    dropped_frame = True
                else:
                    deferred_messages.append(queued_message)
            finally:
                self._send_queue.task_done()

        for queued_message in deferred_messages:
            try:
                self._send_queue.put_nowait(queued_message)
            except queue.Full:
                self._diagnostics.worker_errors += 1
                self._diagnostics.last_error = "Recorder command queue stayed full while preserving commands"
                break

    def _write_worker_messages(self) -> None:
        while not self._stop_event.is_set():
            try:
                message = self._send_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._send_message(message)
            except Exception as exc:
                self._record_error(exc)
                self._stop_event.set()
            finally:
                self._send_queue.task_done()
                self._diagnostics.queue_depth = self._send_queue.qsize()

    def _read_worker_messages(self) -> None:
        while not self._stop_event.is_set():
            try:
                message = receive_message(self._worker_read_fd)
            except IPCClosedError:
                self._stop_event.set()
                return
            except Exception as exc:
                self._record_error(exc)
                self._stop_event.set()
                return
            self._apply_worker_diagnostics(message.get("diagnostics", {}))
            if message.get("type") == "error":
                self._diagnostics.last_error = str(message.get("error", "unknown recorder worker error"))

    def _send_message(self, message: dict[str, Any]) -> None:
        if self._worker_write_fd is None:
            raise RuntimeError("LeRobot recorder worker is not connected")
        with self._send_lock:
            send_message(self._worker_write_fd, message)

    def _apply_worker_diagnostics(self, diagnostics: dict[str, Any]) -> None:
        for field_name in (
            "frames_written",
            "saved_episodes",
            "discarded_episodes",
            "empty_save_requests",
            "worker_errors",
            "last_error",
        ):
            if field_name in diagnostics:
                setattr(self._diagnostics, field_name, diagnostics[field_name])

    def _record_error(self, exc: Exception) -> None:
        self._diagnostics.worker_errors += 1
        self._diagnostics.last_error = f"{type(exc).__name__}: {exc}"

    def _wait_for_send_queue(self) -> None:
        deadline = time.monotonic() + WORKER_SHUTDOWN_TIMEOUT_S
        while self._send_queue.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(0.05)

    def _wait_for_worker_shutdown(self) -> None:
        if self._worker_process is None:
            return
        try:
            self._worker_process.wait(timeout=WORKER_SHUTDOWN_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            self._record_error(RuntimeError("LeRobot recorder worker did not exit before timeout"))
            self._worker_process.terminate()

    def _stop_worker_after_failed_start(self) -> None:
        self._stop_event.set()
        if self._worker_process is not None and self._worker_process.poll() is None:
            self._worker_process.terminate()
        self._close_pipes()

    def _close_pipes(self) -> None:
        for fd in (self._worker_read_fd, self._worker_write_fd):
            if fd is None:
                continue
            try:
                os.close(fd)
            except OSError:
                pass
        self._worker_read_fd = None
        self._worker_write_fd = None

    def _drain_worker_stderr(self) -> None:
        if self._worker_process is None or self._worker_process.stderr is None:
            return
        for line in self._worker_process.stderr:
            print(f"[Recording worker] {line.rstrip()}")


def _build_worker_env(project_root: str) -> dict[str, str]:
    env = os.environ.copy()
    for key in (
        "PYTHONHOME",
        "PYTHONEXECUTABLE",
        "PYTHONNOUSERSITE",
        "PYTHONSAFEPATH",
        "__PYVENV_LAUNCHER__",
    ):
        env.pop(key, None)
    env["PYTHONPATH"] = project_root
    return env
