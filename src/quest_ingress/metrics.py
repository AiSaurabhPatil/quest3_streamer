from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time

from .message_types import QuestPacket


def _percentile(values, fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]

    position = float(fraction) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]

    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


@dataclass
class TransportMetricsSnapshot:
    label: str
    packet_rate_hz: float
    dropped_packets: int
    out_of_order_packets: int
    invalid_sequences: int
    drop_percent: float
    average_packet_size_bytes: float
    jitter_ms: float
    age_p50_ms: float | None
    age_p95_ms: float | None
    vpn_hop_p50_ms: float | None
    vpn_hop_p95_ms: float | None

    def format_log_line(self) -> str:
        age_p50 = "n/a" if self.age_p50_ms is None else f"{self.age_p50_ms:.0f}ms"
        age_p95 = "n/a" if self.age_p95_ms is None else f"{self.age_p95_ms:.0f}ms"
        vpn_p50 = "n/a" if self.vpn_hop_p50_ms is None else f"{self.vpn_hop_p50_ms:.0f}ms"
        vpn_p95 = "n/a" if self.vpn_hop_p95_ms is None else f"{self.vpn_hop_p95_ms:.0f}ms"
        return (
            f"[Transport][{self.label}] "
            f"rx={self.packet_rate_hz:.1f}Hz "
            f"drop={self.drop_percent:.1f}% "
            f"jitter={self.jitter_ms:.1f}ms "
            f"age_p50={age_p50} "
            f"age_p95={age_p95} "
            f"vpn_p50={vpn_p50} "
            f"vpn_p95={vpn_p95} "
            f"size={self.average_packet_size_bytes:.0f}B "
            f"out_of_order={self.out_of_order_packets} "
            f"invalid={self.invalid_sequences}"
        )


class TransportMetricsTracker:
    def __init__(self, label: str, log_period_s: float = 3.0, window_size: int = 256):
        self.label = label
        self.log_period_s = max(0.25, float(log_period_s))
        self.window_size = max(8, int(window_size))
        self.reset()

    def reset(self) -> None:
        now = time.monotonic()
        self._last_sequence: int | None = None
        self._dropped_packets = 0
        self._out_of_order_packets = 0
        self._invalid_sequences = 0
        self._packet_count = 0
        self._total_bytes = 0
        self._last_receive_monotonic_s: float | None = None
        self._last_log_monotonic_s = now
        self._packet_count_at_log = 0
        self._bytes_at_log = 0
        self._intervals_s: deque[float] = deque(maxlen=self.window_size)
        self._ages_ms: deque[float] = deque(maxlen=self.window_size)
        self._vpn_hop_ms: deque[float] = deque(maxlen=self.window_size)

    def record(
        self,
        packet: QuestPacket,
        payload_size_bytes: int,
        receive_monotonic_s: float | None = None,
        receive_epoch_ms: float | None = None,
    ) -> None:
        now_monotonic_s = time.monotonic() if receive_monotonic_s is None else receive_monotonic_s
        now_epoch_ms = time.time() * 1000.0 if receive_epoch_ms is None else receive_epoch_ms

        if packet.sequence is None:
            self._invalid_sequences += 1
        else:
            if self._last_sequence is not None:
                if packet.sequence > self._last_sequence + 1:
                    self._dropped_packets += packet.sequence - self._last_sequence - 1
                elif packet.sequence <= self._last_sequence:
                    self._out_of_order_packets += 1
            if self._last_sequence is None or packet.sequence > self._last_sequence:
                self._last_sequence = packet.sequence

        if self._last_receive_monotonic_s is not None:
            self._intervals_s.append(now_monotonic_s - self._last_receive_monotonic_s)
        self._last_receive_monotonic_s = now_monotonic_s

        self._packet_count += 1
        self._total_bytes += int(payload_size_bytes)

        if packet.transport.client_epoch_ms is not None:
            self._ages_ms.append(max(0.0, now_epoch_ms - packet.transport.client_epoch_ms))
        if packet.transport.ingress_receive_epoch_ms is not None:
            self._vpn_hop_ms.append(
                max(0.0, now_epoch_ms - packet.transport.ingress_receive_epoch_ms)
            )

    def maybe_snapshot(self, now_monotonic_s: float | None = None) -> TransportMetricsSnapshot | None:
        now_monotonic_s = time.monotonic() if now_monotonic_s is None else now_monotonic_s
        if now_monotonic_s - self._last_log_monotonic_s < self.log_period_s:
            return None
        snapshot = self.snapshot(now_monotonic_s)
        self._last_log_monotonic_s = now_monotonic_s
        self._packet_count_at_log = self._packet_count
        self._bytes_at_log = self._total_bytes
        return snapshot

    def snapshot(self, now_monotonic_s: float | None = None) -> TransportMetricsSnapshot:
        now_monotonic_s = time.monotonic() if now_monotonic_s is None else now_monotonic_s
        elapsed = max(1e-6, now_monotonic_s - self._last_log_monotonic_s)
        packets_since_log = self._packet_count - self._packet_count_at_log
        bytes_since_log = self._total_bytes - self._bytes_at_log
        packet_rate_hz = packets_since_log / elapsed
        average_packet_size_bytes = (
            bytes_since_log / packets_since_log if packets_since_log > 0 else 0.0
        )
        total_expected = self._packet_count + self._dropped_packets
        drop_percent = 100.0 * self._dropped_packets / total_expected if total_expected else 0.0

        jitter_ms = 0.0
        if len(self._intervals_s) >= 2:
            average_interval = sum(self._intervals_s) / len(self._intervals_s)
            jitter_ms = (
                sum(abs(interval - average_interval) for interval in self._intervals_s)
                / len(self._intervals_s)
            ) * 1000.0

        return TransportMetricsSnapshot(
            label=self.label,
            packet_rate_hz=packet_rate_hz,
            dropped_packets=self._dropped_packets,
            out_of_order_packets=self._out_of_order_packets,
            invalid_sequences=self._invalid_sequences,
            drop_percent=drop_percent,
            average_packet_size_bytes=average_packet_size_bytes,
            jitter_ms=jitter_ms,
            age_p50_ms=_percentile(self._ages_ms, 0.50),
            age_p95_ms=_percentile(self._ages_ms, 0.95),
            vpn_hop_p50_ms=_percentile(self._vpn_hop_ms, 0.50),
            vpn_hop_p95_ms=_percentile(self._vpn_hop_ms, 0.95),
        )
