from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from src.robot_adapters import RobotAction


@dataclass(frozen=True)
class PolicyObservation:
    joint_positions: np.ndarray
    monotonic_time_s: float


class PolicyActionProvider(Protocol):
    def reset(self) -> None: ...

    def get_action(self, observation: PolicyObservation) -> RobotAction | None: ...


class NullPolicyActionProvider:
    def reset(self) -> None:
        return None

    def get_action(self, observation: PolicyObservation) -> RobotAction | None:
        return None
