#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path
import json


def _read_episode_scene_sidecar(dataset_root: Path, episode_index: int) -> dict | None:
    """Read the per-episode domain randomization sidecar JSON if it exists.

    The sidecar is written by LeRobotWorkerProcess._write_episode_scene_sidecar
    at ``<dataset_root>/meta/episodes/scene_ep_{N:06d}.json`` and contains the
    serialized DomainRandomizationSample for that episode.
    """
    sidecar_path = dataset_root / "meta" / "episodes" / f"scene_ep_{episode_index:06d}.json"
    if not sidecar_path.exists():
        return None
    try:
        with sidecar_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        return data.get("domain_randomization")
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[extract_states] Warning: could not read sidecar {sidecar_path}: {exc}", file=sys.stderr)
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--dataset-dir", default="datasets")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    
    try:
        import lerobot.common.datasets.lerobot_dataset as dataset_module
    except ImportError:
        import lerobot.datasets.lerobot_dataset as dataset_module
        
    dataset_root = Path(args.dataset_dir).expanduser().resolve() / args.repo_id
    dataset = dataset_module.LeRobotDataset(repo_id=args.repo_id, root=dataset_root)
    
    episodes_data = []
    
    frames_by_ep = {}
    
    for global_idx in range(len(dataset)):
        frame = dataset[global_idx]
        ep_idx = frame.get("episode_index", 0)
        if hasattr(ep_idx, "item"):
            ep_idx = ep_idx.item()
            
        if ep_idx not in frames_by_ep:
            frames_by_ep[ep_idx] = []
            
        # Extract state and action
        state = frame.get("observation.state", None)
        if hasattr(state, "numpy"):
            state = state.numpy()
        
        action = frame.get("action", None)
        if hasattr(action, "numpy"):
            action = action.numpy()
            
        frame_data = {}
        if state is not None:
            frame_data["observation.state"] = state.tolist() if hasattr(state, "tolist") else state
        if action is not None:
            frame_data["action"] = action.tolist() if hasattr(action, "tolist") else action
            
        frames_by_ep[ep_idx].append(frame_data)
        
    for ep_idx, frames in sorted(frames_by_ep.items()):
        # Read the sidecar JSON for this episode's domain randomization config.
        domain_randomization = _read_episode_scene_sidecar(dataset_root, ep_idx)
        episode_entry = {
            "episode_index": ep_idx,
            "length": len(frames),
            "frames": frames,
        }
        if domain_randomization is not None:
            episode_entry["domain_randomization"] = domain_randomization
        episodes_data.append(episode_entry)
        
    with open(args.output, "w") as f:
        json.dump({
            "fps": dataset.fps,
            "episodes": episodes_data
        }, f)
        
if __name__ == "__main__":
    main()

