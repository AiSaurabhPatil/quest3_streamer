#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import shutil
import subprocess
from typing import Any
from uuid import uuid4


@dataclass(frozen=True)
class ValidationIssue:
    level: str
    message: str


def _load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as file:
        return [json.loads(line) for line in file if line.strip()]


def _dataset_root(root: Path, repo_id: str) -> Path:
    if (root / "meta" / "info.json").exists():
        return root
    return root / repo_id


def _episode_chunk(episode_index: int, chunks_size: int) -> int:
    return episode_index // max(1, chunks_size)


def _episode_parquet_path(dataset_root: Path, info: dict[str, Any], episode_index: int) -> Path:
    chunks_size = int(info.get("chunks_size", 1000))
    rel = str(info.get("data_path", "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"))
    return dataset_root / rel.format(
        episode_chunk=_episode_chunk(episode_index, chunks_size),
        episode_index=episode_index,
    )


def _video_path(dataset_root: Path, info: dict[str, Any], episode_index: int, video_key: str) -> Path:
    chunks_size = int(info.get("chunks_size", 1000))
    rel = str(info.get("video_path", "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"))
    return dataset_root / rel.format(
        episode_chunk=_episode_chunk(episode_index, chunks_size),
        episode_index=episode_index,
        video_key=video_key,
    )


def _read_parquet(path: Path):
    try:
        import pandas as pd
    except ImportError as exc:
        raise RuntimeError("pandas is required for parquet validation") from exc
    return pd.read_parquet(path)


def _ffprobe_video(path: Path) -> dict[str, str]:
    if shutil.which("ffprobe") is None:
        raise RuntimeError("ffprobe is not installed")
    result = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=codec_name,width,height,r_frame_rate,avg_frame_rate,nb_frames,duration",
            "-of",
            "default=noprint_wrappers=1",
            str(path),
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    values: dict[str, str] = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value
    return values


def _rate_to_float(value: str) -> float:
    if "/" not in value:
        return float(value)
    num, den = value.split("/", 1)
    return float(num) / float(den)


def validate_dataset(root: Path, repo_id: str, episode_index: int) -> list[ValidationIssue]:
    dataset_root = _dataset_root(root, repo_id)
    issues: list[ValidationIssue] = []

    required = (
        dataset_root / "meta" / "info.json",
        dataset_root / "meta" / "episodes.jsonl",
        dataset_root / "meta" / "episodes_stats.jsonl",
        dataset_root / "meta" / "tasks.jsonl",
        dataset_root / "data",
    )
    for path in required:
        if not path.exists():
            issues.append(ValidationIssue("error", f"Missing required path: {path}"))
    if any(issue.level == "error" for issue in issues):
        return issues

    info = _load_json(dataset_root / "meta" / "info.json")
    if info.get("codebase_version") != "v2.1":
        issues.append(ValidationIssue("error", f"Expected v2.1 dataset, got {info.get('codebase_version')}"))

    episodes = _load_jsonl(dataset_root / "meta" / "episodes.jsonl")
    episode = next((item for item in episodes if int(item.get("episode_index", -1)) == episode_index), None)
    if episode is None:
        issues.append(ValidationIssue("error", f"Episode {episode_index} is missing from episodes.jsonl"))
        return issues

    parquet_path = _episode_parquet_path(dataset_root, info, episode_index)
    if not parquet_path.exists():
        issues.append(ValidationIssue("error", f"Missing parquet file: {parquet_path}"))
        return issues

    try:
        df = _read_parquet(parquet_path)
    except Exception as exc:
        issues.append(ValidationIssue("error", f"Could not read parquet: {exc}"))
        return issues

    expected_length = int(episode.get("length", -1))
    if len(df) != expected_length:
        issues.append(ValidationIssue("error", f"Parquet rows {len(df)} != episode length {expected_length}"))

    total_frames = int(info.get("total_frames", -1))
    if len(episodes) == 1 and len(df) != total_frames:
        issues.append(ValidationIssue("error", f"Parquet rows {len(df)} != info total_frames {total_frames}"))

    features = dict(info.get("features", {}))
    for key in ("observation.state", "action", "timestamp", "frame_index", "episode_index", "index", "task_index"):
        if key not in df.columns:
            issues.append(ValidationIssue("error", f"Missing parquet column: {key}"))

    for key in ("observation.state", "action"):
        if key in df.columns and key in features and len(df):
            expected_shape = features[key].get("shape", [])
            expected_width = int(expected_shape[0]) if expected_shape else None
            actual_width = len(df[key].iloc[0])
            if expected_width is not None and actual_width != expected_width:
                issues.append(ValidationIssue("error", f"{key} width {actual_width} != feature shape {expected_width}"))
            for row_index, vector in enumerate(df[key]):
                values = [float(value) for value in vector]
                if any(not math.isfinite(value) for value in values):
                    issues.append(ValidationIssue("error", f"{key} has NaN/inf at row {row_index}"))
                    break

    if {"timestamp", "frame_index"}.issubset(df.columns):
        fps = float(info.get("fps", 0))
        if fps > 0:
            max_error = max(abs(float(ts) - (int(frame) / fps)) for ts, frame in zip(df["timestamp"], df["frame_index"]))
            if max_error > 1e-3:
                issues.append(ValidationIssue("error", f"Timestamp/frame_index drift is {max_error:.6f}s"))

    video_features = {
        key: value for key, value in features.items() if isinstance(value, dict) and value.get("dtype") == "video"
    }
    for key, feature in video_features.items():
        path = _video_path(dataset_root, info, episode_index, key)
        if not path.exists():
            issues.append(ValidationIssue("error", f"Missing video file: {path}"))
            continue
        try:
            probe = _ffprobe_video(path)
        except Exception as exc:
            issues.append(ValidationIssue("warning", f"Could not ffprobe {key}: {exc}"))
            continue
        shape = feature.get("shape", [])
        if len(shape) == 3:
            _, expected_height, expected_width = [int(value) for value in shape]
            if int(probe.get("width", -1)) != expected_width or int(probe.get("height", -1)) != expected_height:
                issues.append(ValidationIssue("error", f"{key} video resolution does not match feature shape"))
        if "nb_frames" in probe and probe["nb_frames"].isdigit() and int(probe["nb_frames"]) != expected_length:
            issues.append(ValidationIssue("error", f"{key} video frames {probe['nb_frames']} != episode length {expected_length}"))
        if "avg_frame_rate" in probe:
            video_fps = _rate_to_float(probe["avg_frame_rate"])
            if abs(video_fps - float(info.get("fps", video_fps))) > 0.01:
                issues.append(ValidationIssue("error", f"{key} fps {video_fps:.3f} != dataset fps {info.get('fps')}"))

    return issues


def _load_lerobot_dataset(repo_id: str, root: Path, episode_index: int):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    return LeRobotDataset(repo_id, root=root, episodes=[episode_index])


def _to_hwc_uint8(image):
    import numpy as np
    import torch

    if isinstance(image, torch.Tensor):
        image = image.detach().cpu()
        if image.dtype == torch.float32 or image.max() <= 1:
            image = (image * 255).clamp(0, 255).to(torch.uint8)
        if image.ndim == 3 and image.shape[0] < image.shape[1]:
            image = image.permute(1, 2, 0)
        return image.numpy()
    array = np.asarray(image)
    if array.ndim == 3 and array.shape[0] < array.shape[1]:
        array = array.transpose(1, 2, 0)
    if array.dtype != np.uint8:
        array = np.clip(array * 255 if array.max() <= 1 else array, 0, 255).astype(np.uint8)
    return array


def visualize_dataset(
    *,
    root: Path,
    repo_id: str,
    episode_index: int,
    save_rrd: Path | None,
    serve: bool,
    grpc_port: int,
) -> Path | None:
    import rerun as rr
    import rerun.blueprint as rrb

    dataset = _load_lerobot_dataset(repo_id, root, episode_index)
    blueprint = rrb.Blueprint(
        rrb.Grid(
            rrb.TimeSeriesView(origin="/action", name="Action", plot_legend=rrb.PlotLegend(visible=True)),
            rrb.TimeSeriesView(origin="/action_minus_state", name="Action - State", plot_legend=rrb.PlotLegend(visible=True)),
            rrb.Spatial2DView(origin="/observation/images/head_camera", name="Head Camera"),
            rrb.Spatial2DView(origin="/observation/images/left_wrist_camera", name="Left Wrist Camera"),
            rrb.Spatial2DView(origin="/observation/images/right_wrist_camera", name="Right Wrist Camera"),
            rrb.TimeSeriesView(origin="/state", name="State", plot_legend=rrb.PlotLegend(visible=True)),
            grid_columns=3,
        ),
        auto_views=False,
        collapse_panels=True,
    )
    rr.init(
        f"{repo_id}/episode_{episode_index}",
        recording_id=str(uuid4()),
        spawn=not save_rrd and not serve,
        default_blueprint=blueprint,
    )
    if serve:
        rr.serve_grpc(grpc_port=grpc_port)

    state_names = dataset.features.get("observation.state", {}).get("names") or []
    action_names = dataset.features.get("action", {}).get("names") or []
    camera_keys = list(getattr(dataset.meta, "camera_keys", []))
    scalar_cls = getattr(rr, "Scalars", rr.Scalar)
    series_line_cls = getattr(rr, "SeriesLine", None)

    if series_line_cls is not None:
        for name in state_names:
            rr.log(f"state/{name}", series_line_cls(name=name), static=True)
        for name in action_names:
            rr.log(f"action/{name}", series_line_cls(name=name), static=True)
            rr.log(f"action_minus_state/{name}", series_line_cls(name=name), static=True)

    for sample in dataset:
        frame_index = int(sample["frame_index"])
        timestamp = float(sample["timestamp"])
        if hasattr(rr, "set_time"):
            rr.set_time("frame_index", sequence=frame_index)
            rr.set_time("timestamp", timestamp=timestamp)
        else:
            rr.set_time_sequence("frame_index", frame_index)
            rr.set_time_seconds("timestamp", timestamp)

        for key in camera_keys:
            rr.log(key.replace(".", "/"), rr.Image(_to_hwc_uint8(sample[key])))

        if "observation.state" in sample:
            for index, value in enumerate(sample["observation.state"]):
                name = state_names[index] if index < len(state_names) else str(index)
                rr.log(f"state/{name}", scalar_cls(float(value)))
        if "action" in sample:
            for index, value in enumerate(sample["action"]):
                name = action_names[index] if index < len(action_names) else str(index)
                rr.log(f"action/{name}", scalar_cls(float(value)))
        if "observation.state" in sample and "action" in sample:
            for index, (state_value, action_value) in enumerate(zip(sample["observation.state"], sample["action"])):
                name = action_names[index] if index < len(action_names) else str(index)
                rr.log(f"action_minus_state/{name}", scalar_cls(float(action_value - state_value)))

    if save_rrd is not None:
        save_rrd.parent.mkdir(parents=True, exist_ok=True)
        rr.save(save_rrd)
        return save_rrd
    if serve:
        print(f"Rerun gRPC server listening on port {grpc_port}")
    return None


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate and visualize a local LeRobot v2.1 dataset episode.")
    parser.add_argument("--repo-id", required=True, help="Dataset repo id, for example local/quest3-acone.")
    parser.add_argument("--root", type=Path, default=Path("datasets"), help="Dataset root or exact dataset directory.")
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--skip-validation", action="store_true")
    parser.add_argument("--save-rrd", type=Path, default=None)
    parser.add_argument("--serve", action="store_true", help="Serve Rerun over gRPC instead of spawning the viewer.")
    parser.add_argument("--grpc-port", type=int, default=9876)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    root = args.root.expanduser().resolve()

    if not args.skip_validation:
        issues = validate_dataset(root, args.repo_id, args.episode_index)
        for issue in issues:
            print(f"[{issue.level.upper()}] {issue.message}")
        if any(issue.level == "error" for issue in issues):
            return 1
        if not issues:
            print("Validation passed")

    if args.validate_only:
        return 0

    saved = visualize_dataset(
        root=root,
        repo_id=args.repo_id,
        episode_index=args.episode_index,
        save_rrd=args.save_rrd,
        serve=args.serve,
        grpc_port=args.grpc_port,
    )
    if saved is not None:
        print(f"Saved Rerun recording: {saved}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
