from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum, IntEnum
from typing import Mapping

import numpy as np

from .controller_state import ControllerState


class ControlMode(str, Enum):
    CONTINUOUS = "continuous"
    CLUTCHED = "clutched"
    POLICY_INTERVENTION = "policy_intervention"


class ControlSource(IntEnum):
    POLICY = 0
    HUMAN = 1
    MIXED = 2
    HOLD = 3


@dataclass(frozen=True)
class ClutchConfig:
    enabled: bool = False
    button: str = "grip"
    arbitration: str = "per_hand"
    policy_reentry_blend_s: float = 0.2
    analog_threshold: float = 0.1

    @classmethod
    def from_mapping(cls, values: Mapping[str, object] | None) -> "ClutchConfig":
        values = values or {}
        blend_s = values.get("policy_reentry_blend_s")
        if blend_s is None and "policy_reentry_blend_ms" in values:
            blend_s = float(values["policy_reentry_blend_ms"]) / 1000.0
        config = cls(
            enabled=bool(values.get("enabled", False)),
            button=str(values.get("button", "grip")),
            arbitration=str(values.get("arbitration", "per_hand")),
            policy_reentry_blend_s=0.2 if blend_s is None else float(blend_s),
            analog_threshold=float(values.get("analog_threshold", 0.1)),
        )
        config.validate()
        return config

    def validate(self) -> None:
        if self.button not in {"primary", "secondary", "menu", "stick_click", "grip", "squeeze"}:
            raise ValueError(f"Unsupported clutch button '{self.button}'")
        if self.arbitration not in {"global", "per_hand"}:
            raise ValueError(f"Unsupported clutch arbitration '{self.arbitration}'")
        if self.policy_reentry_blend_s < 0.0:
            raise ValueError("policy_reentry_blend_s must be non-negative")
        if self.analog_threshold < 0.0:
            raise ValueError("analog_threshold must be non-negative")

    def with_enabled(self, enabled: bool) -> "ClutchConfig":
        return replace(self, enabled=bool(enabled))


@dataclass(frozen=True)
class EndEffectorPose:
    position_xyz: np.ndarray
    orientation_wxyz: np.ndarray
    valid: bool = True

    def __post_init__(self):
        position = np.asarray(self.position_xyz, dtype=float).reshape(-1)
        if position.size != 3 or not np.all(np.isfinite(position)):
            position = np.zeros(3, dtype=float)
            valid = False
        else:
            valid = bool(self.valid)
        quat = np.asarray(self.orientation_wxyz, dtype=float).reshape(-1)
        if quat.size != 4 or not np.all(np.isfinite(quat)):
            quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
            valid = False
        else:
            norm = float(np.linalg.norm(quat))
            if norm <= 1e-9:
                quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
                valid = False
            else:
                quat = quat / norm
        object.__setattr__(self, "position_xyz", position.astype(float, copy=True))
        object.__setattr__(self, "orientation_wxyz", quat.astype(float, copy=True))
        object.__setattr__(self, "valid", valid)


@dataclass(frozen=True)
class HandClutchInput:
    pressed: bool = False
    rising: bool = False
    falling: bool = False


@dataclass(frozen=True)
class BimanualClutchInput:
    left: HandClutchInput = field(default_factory=HandClutchInput)
    right: HandClutchInput = field(default_factory=HandClutchInput)

    @property
    def any_rising(self) -> bool:
        return self.left.rising or self.right.rising


class ClutchInputMapper:
    def __init__(self, config: ClutchConfig):
        self.config = config
        self._previous = {"left": False, "right": False}

    def reset(self) -> None:
        self._previous = {"left": False, "right": False}

    def update(self, states: Mapping[str, ControllerState | None]) -> BimanualClutchInput:
        if not self.config.enabled:
            self.reset()
            return BimanualClutchInput()

        physical = {
            hand: self._button_pressed(states.get(hand))
            for hand in ("left", "right")
        }
        if self.config.arbitration == "global":
            pressed = physical["left"] or physical["right"]
            previous = self._previous["left"] or self._previous["right"]
            self._previous = {"left": pressed, "right": pressed}
            logical = HandClutchInput(
                pressed=pressed,
                rising=pressed and not previous,
                falling=previous and not pressed,
            )
            return BimanualClutchInput(left=logical, right=logical)

        left = self._edge("left", physical["left"])
        right = self._edge("right", physical["right"])
        return BimanualClutchInput(left=left, right=right)

    def _button_pressed(self, state: ControllerState | None) -> bool:
        if state is None:
            return False
        if self.config.button in {"grip", "squeeze"}:
            return float(getattr(state.axes, "squeeze", 0.0)) > self.config.analog_threshold
        buttons = getattr(state, "buttons", None)
        return bool(getattr(buttons, self.config.button, False))

    def _edge(self, hand: str, pressed: bool) -> HandClutchInput:
        previous = self._previous[hand]
        self._previous[hand] = pressed
        return HandClutchInput(
            pressed=pressed,
            rising=pressed and not previous,
            falling=previous and not pressed,
        )


@dataclass
class ClutchAnchor:
    controller_position_xyz: np.ndarray
    controller_orientation_wxyz: np.ndarray
    controller_orientation_xyzw: np.ndarray
    ee_position_xyz: np.ndarray
    ee_orientation_wxyz: np.ndarray
    engaged_at_s: float


@dataclass(frozen=True)
class HandInterventionStatus:
    active: bool = False
    forced_hold: bool = False


@dataclass(frozen=True)
class InterventionStatus:
    left: HandInterventionStatus = field(default_factory=HandInterventionStatus)
    right: HandInterventionStatus = field(default_factory=HandInterventionStatus)
    intervention_id: int = -1

    @property
    def active(self) -> bool:
        return self.left.active or self.right.active
