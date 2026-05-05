from __future__ import annotations

import time

from src.robot_adapters import OpenArmAdapter


class ControlMetricsReporter:
    def __init__(self, log_period_s: float = 3.0):
        self.log_period_s = max(0.5, float(log_period_s))
        self._last_log_s = time.monotonic()
        self._ready_frames = 0
        self._stale_frames = 0
        self._last_left_success = 0
        self._last_left_fail = 0
        self._last_left_fallback = 0
        self._last_right_success = 0
        self._last_right_fail = 0
        self._last_right_fallback = 0

    def record(self, session_update, adapter: OpenArmAdapter, logger) -> None:
        self._ready_frames += 1
        if session_update.left_state.stale or session_update.right_state.stale:
            self._stale_frames += 1

        now_s = time.monotonic()
        if now_s - self._last_log_s < self.log_period_s:
            return

        diagnostics = adapter.get_diagnostics()
        left_success = diagnostics.counters.get("left_ik_success", 0)
        left_fail = diagnostics.counters.get("left_ik_fail", 0)
        left_fallback = diagnostics.counters.get("left_orientation_fallback", 0)
        right_success = diagnostics.counters.get("right_ik_success", 0)
        right_fail = diagnostics.counters.get("right_ik_fail", 0)
        right_fallback = diagnostics.counters.get("right_orientation_fallback", 0)

        delta_left_success = left_success - self._last_left_success
        delta_left_fail = left_fail - self._last_left_fail
        delta_left_fallback = left_fallback - self._last_left_fallback
        delta_right_success = right_success - self._last_right_success
        delta_right_fail = right_fail - self._last_right_fail
        delta_right_fallback = right_fallback - self._last_right_fallback

        elapsed = max(1e-6, now_s - self._last_log_s)
        loop_rate_hz = self._ready_frames / elapsed
        left_success_pct = _success_percent(delta_left_success, delta_left_fail)
        right_success_pct = _success_percent(delta_right_success, delta_right_fail)
        left_browser_age_ms = _age_ms(session_update.left_state.client_epoch_ms)
        right_browser_age_ms = _age_ms(session_update.right_state.client_epoch_ms)

        logger.info(
            "[Control] "
            f"loop={loop_rate_hz:.1f}Hz "
            f"left_success={left_success_pct:.1f}% "
            f"right_success={right_success_pct:.1f}% "
            f"left_orientation_fallback={delta_left_fallback} "
            f"right_orientation_fallback={delta_right_fallback} "
            f"stale={self._stale_frames} "
            f"left_seq={session_update.left_state.sequence} "
            f"right_seq={session_update.right_state.sequence} "
            f"left_age_ms={session_update.left_state.last_age_ms or 0.0:.0f} "
            f"right_age_ms={session_update.right_state.last_age_ms or 0.0:.0f} "
            f"left_browser_age_ms={left_browser_age_ms:.0f} "
            f"right_browser_age_ms={right_browser_age_ms:.0f}"
        )

        self._last_log_s = now_s
        self._ready_frames = 0
        self._stale_frames = 0
        self._last_left_success = left_success
        self._last_left_fail = left_fail
        self._last_left_fallback = left_fallback
        self._last_right_success = right_success
        self._last_right_fail = right_fail
        self._last_right_fallback = right_fallback


def _success_percent(success: int, fail: int) -> float:
    total = success + fail
    return 100.0 if total == 0 else 100.0 * success / total


def _age_ms(epoch_ms: float | None) -> float:
    if epoch_ms is None:
        return 0.0
    return max(0.0, time.time() * 1000.0 - float(epoch_ms))
