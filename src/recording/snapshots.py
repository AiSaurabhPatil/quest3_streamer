from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class RecordingFrameSnapshot:
    state: np.ndarray
    action: np.ndarray
    cameras: dict[str, np.ndarray] = field(default_factory=dict)
    task: str = ""
    monotonic_time_s: float = 0.0
    sequence: int | None = None


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
