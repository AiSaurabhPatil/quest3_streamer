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
