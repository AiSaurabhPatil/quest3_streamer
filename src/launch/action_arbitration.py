from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from src.robot_adapters import RobotAction
from src.teleop_core import ControlMode, ControlSource, InterventionStatus


@dataclass(frozen=True)
class ActionArbiterConfig:
    mode: ControlMode
    arbitration: str = "per_hand"
    policy_reentry_blend_s: float = 0.2


@dataclass(frozen=True)
class ArbitrationResult:
    executed_action: RobotAction
    human_action: RobotAction | None
    policy_action: RobotAction | None
    control_source: ControlSource
    left_human: bool
    right_human: bool
    intervention_id: int
    human_action_valid: bool
    policy_action_valid: bool
    reentry_blend_active: bool


@dataclass
class _ReleaseBlend:
    start_s: float
    start_values: np.ndarray


class BimanualActionArbiter:
    def __init__(
        self,
        *,
        config: ActionArbiterConfig,
        left_indices: Sequence[int],
        right_indices: Sequence[int],
    ):
        self.config = config
        self.left_indices = np.asarray(left_indices, dtype=int)
        self.right_indices = np.asarray(right_indices, dtype=int)
        self._last_executed: np.ndarray | None = None
        self._previous_left_human = False
        self._previous_right_human = False
        self._left_release_blend: _ReleaseBlend | None = None
        self._right_release_blend: _ReleaseBlend | None = None

    def reset(self) -> None:
        self._last_executed = None
        self._previous_left_human = False
        self._previous_right_human = False
        self._left_release_blend = None
        self._right_release_blend = None

    def select(
        self,
        *,
        current_positions: np.ndarray,
        human_action: RobotAction | None,
        policy_action: RobotAction | None,
        intervention: InterventionStatus,
        now_s: float,
    ) -> ArbitrationResult:
        current = np.asarray(current_positions, dtype=float).reshape(-1).copy()
        expected_size = current.size
        human_valid = self._valid(human_action, expected_size)
        policy_valid = self._valid(policy_action, expected_size)
        human_values = None if not human_valid else human_action.joint_positions.copy()
        policy_values = None if not policy_valid else policy_action.joint_positions.copy()
        hold = current.copy() if self._last_executed is None else self._last_executed.copy()

        if self.config.mode == ControlMode.CONTINUOUS:
            executed = human_values.copy() if human_values is not None else hold
            source = ControlSource.HUMAN if human_values is not None else ControlSource.HOLD
            self._last_executed = executed.copy()
            return ArbitrationResult(
                executed_action=RobotAction(executed),
                human_action=human_action if human_valid else None,
                policy_action=policy_action if policy_valid else None,
                control_source=source,
                left_human=False,
                right_human=False,
                intervention_id=-1,
                human_action_valid=human_valid,
                policy_action_valid=policy_valid,
                reentry_blend_active=False,
            )

        if self.config.arbitration == "global":
            left_human = right_human = intervention.active
        else:
            left_human = intervention.left.active
            right_human = intervention.right.active

        if intervention.left.forced_hold:
            left_human = False
        if intervention.right.forced_hold:
            right_human = False

        executed = hold.copy()
        if self.config.mode == ControlMode.POLICY_INTERVENTION and policy_values is not None:
            executed = policy_values.copy()

        if intervention.left.forced_hold:
            executed[self.left_indices] = hold[self.left_indices]
        if intervention.right.forced_hold:
            executed[self.right_indices] = hold[self.right_indices]

        if left_human and human_values is not None:
            executed[self.left_indices] = human_values[self.left_indices]
        if right_human and human_values is not None:
            executed[self.right_indices] = human_values[self.right_indices]

        self._capture_release_blends(left_human, right_human, hold, now_s)
        reentry_active = False
        if self.config.mode == ControlMode.POLICY_INTERVENTION and policy_values is not None:
            reentry_active |= self._apply_release_blend(
                "_left_release_blend",
                self.left_indices,
                executed,
                policy_values,
                now_s,
                skip=intervention.left.forced_hold or left_human,
            )
            reentry_active |= self._apply_release_blend(
                "_right_release_blend",
                self.right_indices,
                executed,
                policy_values,
                now_s,
                skip=intervention.right.forced_hold or right_human,
            )

        if left_human and right_human:
            source = ControlSource.HUMAN
        elif left_human or right_human or reentry_active:
            source = ControlSource.MIXED
        elif self.config.mode == ControlMode.POLICY_INTERVENTION and policy_values is not None and not (
            intervention.left.forced_hold or intervention.right.forced_hold
        ):
            source = ControlSource.POLICY
        else:
            source = ControlSource.HOLD

        self._previous_left_human = left_human
        self._previous_right_human = right_human
        self._last_executed = executed.copy()
        return ArbitrationResult(
            executed_action=RobotAction(executed.copy()),
            human_action=human_action if human_valid else None,
            policy_action=policy_action if policy_valid else None,
            control_source=source,
            left_human=left_human,
            right_human=right_human,
            intervention_id=intervention.intervention_id,
            human_action_valid=human_valid,
            policy_action_valid=policy_valid,
            reentry_blend_active=reentry_active,
        )

    @staticmethod
    def _valid(action: RobotAction | None, expected_size: int) -> bool:
        if action is None:
            return False
        values = np.asarray(action.joint_positions, dtype=float).reshape(-1)
        return values.size == expected_size and bool(np.all(np.isfinite(values)))

    def _capture_release_blends(
        self,
        left_human: bool,
        right_human: bool,
        hold: np.ndarray,
        now_s: float,
    ) -> None:
        if self._previous_left_human and not left_human:
            self._left_release_blend = _ReleaseBlend(now_s, hold[self.left_indices].copy())
        if self._previous_right_human and not right_human:
            self._right_release_blend = _ReleaseBlend(now_s, hold[self.right_indices].copy())

    def _apply_release_blend(
        self,
        attr: str,
        indices: np.ndarray,
        executed: np.ndarray,
        policy_values: np.ndarray,
        now_s: float,
        *,
        skip: bool,
    ) -> bool:
        blend = getattr(self, attr)
        if blend is None or skip:
            return False
        blend_s = max(0.0, float(self.config.policy_reentry_blend_s))
        alpha = 1.0 if blend_s <= 0.0 else float(np.clip((now_s - blend.start_s) / blend_s, 0.0, 1.0))
        executed[indices] = blend.start_values + (policy_values[indices] - blend.start_values) * alpha
        if alpha >= 1.0:
            setattr(self, attr, None)
            return False
        return True
