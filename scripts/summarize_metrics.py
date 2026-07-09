#!/usr/bin/env python3
"""
Summarize teleop latency metrics from a captured log stream.

The teleop stack already logs two metric lines every few seconds:

    [Transport][direct] rx=89.8Hz drop=0.2% jitter=4.1ms age_p50=31ms age_p95=74ms size=512B
    [Control] loop=58.2Hz left_success=99.0% right_success=98.5% ... left_age_ms=33 right_age_ms=35 left_browser_age_ms=120 ...

This tool parses those lines from a file (or stdin) and prints per-metric
min / p50 / p95 / max so you can compare a "before" and "after" capture when
tuning for latency. It performs no side effects and writes nothing.

Usage:
    python scripts/summarize_metrics.py path/to/log.txt
    some-command 2>&1 | python scripts/summarize_metrics.py
    python scripts/summarize_metrics.py file1.txt file2.txt --labels before after
"""

from __future__ import annotations

import argparse
import re
import statistics
import sys
from pathlib import Path


# A metric is identified by a regex that captures a floating point value after
# a named key. We search each log line for any of these patterns.
METRIC_PATTERNS: dict[str, re.Pattern[str]] = {
    # [Transport] line
    "rx_hz": re.compile(r"\brx=([\d.]+)Hz\b"),
    "drop_pct": re.compile(r"\bdrop=([\d.]+)%"),
    "jitter_ms": re.compile(r"\bjitter=([\d.]+)ms\b"),
    "age_p50_ms": re.compile(r"\bage_p50=([\d.]+)ms\b"),
    "age_p95_ms": re.compile(r"\bage_p95=([\d.]+)ms\b"),
    # [Control] line
    "loop_hz": re.compile(r"\bloop=([\d.]+)Hz\b"),
    "left_age_ms": re.compile(r"\bleft_age_ms=([\d.]+)\b"),
    "right_age_ms": re.compile(r"\bright_age_ms=([\d.]+)\b"),
    "left_browser_age_ms": re.compile(r"\bleft_browser_age_ms=([\d.]+)\b"),
    "right_browser_age_ms": re.compile(r"\bright_browser_age_ms=([\d.]+)\b"),
    "left_success_pct": re.compile(r"\bleft_success=([\d.]+)%"),
    "right_success_pct": re.compile(r"\bright_success=([\d.]+)%"),
}


def _percentile(values: list[float], pct: float) -> float:
    """Linear-interpolation percentile (matches numpy default)."""
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (pct / 100.0) * (len(ordered) - 1)
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    frac = rank - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * frac


def parse_stream(lines) -> dict[str, list[float]]:
    collected: dict[str, list[float]] = {name: [] for name in METRIC_PATTERNS}
    for line in lines:
        for name, pattern in METRIC_PATTERNS.items():
            match = pattern.search(line)
            if match:
                try:
                    collected[name].append(float(match.group(1)))
                except ValueError:
                    pass
    return collected


def summarize(collected: dict[str, list[float]]) -> list[tuple[str, str, str, str, str, str]]:
    rows: list[tuple[str, str, str, str, str, str]] = []
    for name in METRIC_PATTERNS:
        values = collected.get(name, [])
        if not values:
            rows.append((name, "-", "-", "-", "-", "-"))
            continue
        rows.append(
            (
                name,
                f"{min(values):.1f}",
                f"{_percentile(values, 50):.1f}",
                f"{_percentile(values, 95):.1f}",
                f"{max(values):.1f}",
                str(len(values)),
            )
        )
    return rows


def _read_lines(source: str):
    if source == "-":
        yield from sys.stdin
        return
    path = Path(source)
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        yield from handle


def _print_table(title: str, rows: list[tuple[str, str, str, str, str, str]]) -> None:
    print(f"\n=== {title} ===")
    header = ("metric", "min", "p50", "p95", "max", "n")
    widths = [max(len(str(row[i])) for row in rows + [header]) for i in range(6)]
    fmt = "  ".join(f"{{:<{w}}}" for w in widths)
    print(fmt.format(*header))
    print(fmt.format(*("-" * w for w in widths)))
    for row in rows:
        print(fmt.format(*row))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "sources",
        nargs="+",
        help="Log file(s) to parse. Use '-' for stdin. Multiple files are summarized side by side.",
    )
    parser.add_argument(
        "--labels",
        nargs="+",
        help="Optional labels for each source (defaults to the filenames).",
    )
    args = parser.parse_args(argv)

    labels = args.labels or args.sources
    if len(labels) != len(args.sources):
        parser.error("--labels count must match number of sources")

    for source, label in zip(args.sources, labels):
        collected = parse_stream(_read_lines(source))
        rows = summarize(collected)
        _print_table(label, rows)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
