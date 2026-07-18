#!/usr/bin/env python3
"""Deferred offline renderer for LeRobot datasets.

Replays recorded joint trajectories inside Isaac Sim and renders high-quality
camera images that were skipped during real-time teleop to avoid GPU latency.

Three correctness fixes are implemented here:

  Bug 1 — Slow replay speed
    The live control loop ran at ``target_control_rate_hz`` (e.g. 120 Hz) while
    recording only at ``fps`` (e.g. 30 fps).  Each saved frame represents
    ``round(hz/fps)`` physics steps of real motion.  The renderer must advance
    physics the same number of steps (without rendering) before capturing the
    rendered frame, so the robot servo reaches the commanded position at the
    same rate it did during teleop.

  Bug 2 — No reset between episodes
    ``isaac_app.reset_world()`` is called at the start of every episode so all
    objects return to their USD default poses before replaying the next
    trajectory.

  Bug 3 — Domain randomization not replayed
    If the source dataset has per-episode sidecar JSON files (written during
    teleop by the worker process), the renderer reads the saved
    ``DomainRandomizationSample`` dict and calls
    ``domain_randomizer.apply_randomization_from_dict()`` before the joint
    replay so the scene exactly matches the one the operator saw.
"""
import argparse
import os
import sys
from pathlib import Path
import json
import subprocess
import tempfile
import numpy as np
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.dirname(SCRIPT_DIR)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from isaac_backend.app import IsaacApp


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Deferred Offline Renderer for LeRobot Datasets")
    parser.add_argument("--repo-id", required=True, help="HuggingFace repo ID of the dataset to render")
    parser.add_argument("--output-repo-id", required=True, help="Output repo ID")
    parser.add_argument("--dataset-dir", default="datasets", help="Root directory for datasets")
    parser.add_argument("--robot-type", default="openarm", help="Robot type name")
    parser.add_argument("--venv-python", required=True, help="Path to venv python to run extraction script")
    parser.add_argument(
        "--physics-substeps",
        type=int,
        default=0,
        help=(
            "Number of physics steps to run (without rendering) before each rendered frame. "
            "0 = auto-detect from target_control_rate_hz / fps."
        ),
    )
    return parser


def main(argv=None):
    args = _build_parser().parse_args(argv)

    dataset_root = Path(args.dataset_dir).expanduser().resolve() / args.repo_id
    if not dataset_root.exists():
        print(f"Error: Dataset {dataset_root} does not exist.")
        return 1
        
    output_dataset_root = Path(args.dataset_dir).expanduser().resolve() / args.output_repo_id
    if output_dataset_root.exists():
        import shutil
        print(f"[Renderer] Removing existing output dataset: {output_dataset_root}")
        shutil.rmtree(output_dataset_root)

    # Extract states using venv python (so we don't import lerobot in Isaac Sim)
    states_file = tempfile.mktemp(suffix=".json")
    extract_script = os.path.join(SCRIPT_DIR, "extract_states.py")
    
    print(f"[Renderer] Extracting states from dataset using {args.venv_python}...")
    subprocess.check_call([
        args.venv_python, extract_script,
        "--repo-id", args.repo_id,
        "--dataset-dir", args.dataset_dir,
        "--output", states_file
    ])
    
    with open(states_file, "r") as f:
        dataset_info = json.load(f)
        
    os.remove(states_file)

    # Start Isaac Sim headless
    isaac_app = IsaacApp({"headless": True}).start()
    
    # SimulationApp modifies sys.path, so we must force our source dir to the front
    PROJECT_ROOT = os.path.dirname(SRC_DIR)
    if PROJECT_ROOT in sys.path:
        sys.path.remove(PROJECT_ROOT)
    sys.path.insert(0, PROJECT_ROOT)
    
    print(f"[Renderer Debug] PROJECT_ROOT: {PROJECT_ROOT}")
    print(f"[Renderer Debug] sys.path: {sys.path}")
        
    try:
        from src.isaac_backend import CameraManager
        from src.robot_adapters import FFWBG2Adapter, OpenArmAdapter
        from src.recording import RecordingConfig, build_recording_schema, LeRobotEpisodeRecorder, RecordingFrameSnapshot
        from src.config_loader import load_runtime_config, default_project_root
        
        project_root = default_project_root()
        runtime = load_runtime_config(config_path=None, robot=args.robot_type, project_root=project_root)
        
        if args.robot_type == "ffw_bg2":
            adapter = FFWBG2Adapter.from_mapping(runtime.robot, project_root=project_root)
        else:
            adapter = OpenArmAdapter.from_mapping(runtime.robot, project_root=project_root)
            
        print(f"[Renderer] Loading stage from {adapter.usd_path}...")
        isaac_app.load_stage(adapter.usd_path)
        
        print("[Renderer] Loading environment...")
        world = isaac_app.create_world(stage_units_in_meters=1.0)
        
        try:
            adapter.load(world, world.stage)
        except Exception as e:
            print(f"[Renderer ERROR] Failed to load robot: {e}")
            return 1
            
        print("[Renderer] Resetting World...")
        isaac_app.reset_world()

        # ------------------------------------------------------------------ #
        # Bug 1 fix: determine how many physics substeps per rendered frame.  #
        # The live loop ran at target_control_rate_hz; recording captured      #
        # every N-th step at recording_fps. Replay the same N steps without   #
        # rendering so joints reach the commanded position at the right speed. #
        # ------------------------------------------------------------------ #
        recording_fps = int(dataset_info.get("fps", 30))
        target_control_rate_hz = int(
            runtime.main.get("target_control_rate_hz",
                             runtime.main.get("isaac", {}).get("target_control_rate_hz", 120))
        )
        render_every_n_steps = int(
            runtime.main.get("isaac", {}).get("render_every_n_steps", 1)
        )
        if args.physics_substeps > 0:
            # Explicit override from command line.
            physics_substeps = args.physics_substeps
        else:
            # Auto-detect: live loop ran at target_control_rate_hz and rendered
            # every render_every_n_steps steps, so effective render rate is
            # target_control_rate_hz / render_every_n_steps.  The recorded fps
            # may be lower than that (it's gated on will_render), so:
            physics_substeps = max(1, round(target_control_rate_hz / max(1, recording_fps)))
        print(
            f"[Renderer] Physics substeps per frame: {physics_substeps} "
            f"(control_hz={target_control_rate_hz}, fps={recording_fps})"
        )

        # ------------------------------------------------------------------ #
        # Bug 3 fix: set up domain randomizer if the dataset has scene data.  #
        # ------------------------------------------------------------------ #
        domain_randomizer = None
        has_any_domain_randomization = any(
            "domain_randomization" in ep
            for ep in dataset_info.get("episodes", [])
        )
        if has_any_domain_randomization:
            try:
                dr_config = runtime.robot.get("domain_randomization", {})
                print(f"[Renderer Debug] Loaded dr_config: {dr_config}")
                from src.isaac_backend.domain_randomization import DomainRandomizer
                domain_randomizer = DomainRandomizer(world.stage, dr_config)
                domain_randomizer.initialize()
                print(f"[Renderer] Domain randomizer initialized for scene replay. Enabled: {domain_randomizer.enabled}")
            except Exception as exc:
                print(f"[Renderer] Warning: could not initialize domain randomizer: {exc}")
                domain_randomizer = None

        print("[Renderer] Getting joint information...")
        adapter.initialize_joint_mappings()
        
        camera_specs = adapter.get_camera_specs()
        camera_config = dict(runtime.main.get("cameras", {}))
        camera_config["enabled"] = True
        camera_config["log_errors"] = True
        
        camera_manager = CameraManager(
            stage=world.stage,
            camera_specs=camera_specs,
            camera_publishers=None,
            viewport_cameras=adapter.get_viewport_cameras(),
            config=camera_config,
        ).start()
        
        # Give Replicator a few frames to initialize
        for _ in range(30):
            isaac_app.step(render=True)
            camera_manager.update(return_frames=True)
            
        print("[Renderer] Setting up recorder...")
        rec_config_dict = runtime.main.get("recording", {})
        rec_config_dict.update({
            "enabled": True,
            "repo_id": args.output_repo_id,
            "root": args.dataset_dir,
            "fps": dataset_info["fps"],
            "use_videos": True,
            "deferred_rendering": False,
            "verbose": True,
        })
        # Ensure the recording config uses the correct camera names for this robot
        if "cameras" not in rec_config_dict:
            rec_config_dict["cameras"] = {}
        rec_config_dict["cameras"]["include"] = list(camera_specs.keys())
        
        rec_config = RecordingConfig.from_mapping(rec_config_dict, project_root=".")
        
        schema = build_recording_schema(adapter, adapter.config, rec_config)
        
        # Build mapping from recorded state names to articulation joint indices
        state_names = schema.state_spec.names
        gripper_open = float(adapter.config["grippers"]["open_position"])
        gripper_closed = float(adapter.config["grippers"]["closed_position"])
        
        def _build_name_to_indices():
            mapping = {}
            for name in state_names:
                if name == "left_gripper":
                    mapping[name] = adapter.left_gripper_indices
                elif name == "right_gripper":
                    mapping[name] = adapter.right_gripper_indices
                else:
                    try:
                        idx = adapter.articulation.get_dof_index(name)
                        mapping[name] = [idx]
                    except Exception:
                        print(f"[Renderer Warning] Could not find dof index for {name}")
                        mapping[name] = []
            return mapping

        name_to_indices = _build_name_to_indices()

        recorder = LeRobotEpisodeRecorder(
            config=rec_config,
            schema=schema,
            robot_type=adapter.get_recording_robot_type(),
            project_root=project_root,
        ).start()

        # Try/except helper for ArticulationAction import (Isaac Sim version compat)
        try:
            from isaacsim.core.utils.types import ArticulationAction
        except ImportError:
            from omni.isaac.core.utils.types import ArticulationAction

        print(f"[Renderer] Rendering {len(dataset_info['episodes'])} episodes...")
        for episode in dataset_info["episodes"]:
            ep_idx = episode["episode_index"]
            print(f"[Renderer] Starting episode {ep_idx} ({episode['length']} frames)")

            # ---------------------------------------------------------------- #
            # Bug 2 fix: reset the world at the start of every episode so      #
            # objects return to their USD default poses.                        #
            # ---------------------------------------------------------------- #
            isaac_app.stop()
            if domain_randomizer is not None:
                # Bug 3 fix: restore the exact scene state that existed during
                # teleop for this episode before replaying joint trajectories.
                dr_dict = episode.get("domain_randomization")
                if dr_dict is not None:
                    try:
                        domain_randomizer.apply_randomization_from_dict(dr_dict)
                        print(f"[Renderer] Episode {ep_idx}: restored domain randomization")
                    except Exception as exc:
                        print(f"[Renderer] Warning: could not apply domain randomization for ep {ep_idx}: {exc}")
            isaac_app.reset_world()
            if domain_randomizer is not None and domain_randomizer.enabled:
                domain_randomizer.settle(isaac_app.step)

            # Re-build joint index mapping after reset (Isaac Sim may invalidate
            # the articulation physics handle on stop/play).
            if hasattr(adapter, "reinitialize_physics_handles"):
                adapter.reinitialize_physics_handles()
            name_to_indices = _build_name_to_indices()

            # Warm up a few frames so the articulation physics handle is stable.
            for _ in range(10):
                isaac_app.step(render=False)

            for frame_idx, frame_data in enumerate(episode["frames"]):
                state = frame_data.get("observation.state", None)
                action = frame_data.get("action", None)
                
                if action is not None:
                    recorded_action = np.array(action, dtype=np.float32).reshape(-1)
                    
                    # Start with current joints to preserve unrecorded joints
                    current_positions = adapter.get_current_joint_positions()
                    if current_positions is None:
                        current_positions = np.zeros(adapter.articulation.num_dof, dtype=np.float32)
                    else:
                        current_positions = np.array(current_positions, dtype=np.float32)
                        if len(current_positions.shape) == 2:
                            current_positions = current_positions[0]
                    
                    target_pos = current_positions.copy()
                    
                    action_names = schema.action_spec.names
                    for i, name in enumerate(action_names):
                        val = float(recorded_action[i])
                        if name in ("left_gripper", "right_gripper"):
                            # Denormalize gripper scalar
                            val = gripper_open + val * (gripper_closed - gripper_open)
                            
                        for idx in name_to_indices.get(name, []):
                            target_pos[idx] = val
                            
                    pos = target_pos
                    if adapter._joint_positions_are_2d:
                        pos = pos.reshape(1, -1)

                    action_obj = ArticulationAction(joint_positions=pos)
                    if not hasattr(action_obj, "joint_names"):
                        action_obj.joint_names = None
                    adapter.articulation.apply_action(action_obj)

                # ------------------------------------------------------------ #
                # Bug 1 fix: run (physics_substeps - 1) non-render steps so    #
                # the joint servo fully reaches the commanded position before   #
                # we capture the rendered frame.                                #
                # ------------------------------------------------------------ #
                for _ in range(physics_substeps - 1):
                    isaac_app.step(render=False)
                isaac_app.step(render=True)
                
                # Capture RTX images
                camera_frames = camera_manager.update(return_frames=True, rendered=True)
                
                # Wait for futures
                resolved_cameras = {}
                for cam_name, future in camera_frames.items():
                    result = future.result()
                    if result is not None:
                        resolved_cameras[cam_name] = result[1]  # image_rgb
                        if result[1] is None:
                            print(f"[Renderer ERROR] Camera {cam_name} returned None!")
                
                # Save frame
                frame_to_record = RecordingFrameSnapshot(
                    monotonic_time_s=time.time(),
                    cameras=resolved_cameras,
                    state=np.array(state, dtype=np.float32) if state is not None else None,
                    action=np.array(action, dtype=np.float32) if action is not None else None
                )
                recorder.enqueue_frame(frame_to_record)
                
            recorder.save_episode_async()
            print(f"[Renderer] Saved episode {ep_idx}")
            
        print("[Renderer] Finished rendering all episodes!")
        recorder.finalize()
        _recorder_finalized = True

    except Exception as e:
        print(f"[Renderer ERROR] An exception occurred: {e}")
        import traceback
        traceback.print_exc()
        raise e
    finally:
        if 'recorder' in locals() and hasattr(recorder, 'finalize'):
            try:
                if not locals().get('_recorder_finalized', False):
                    print("[Renderer] Waiting for background video encoding to finish...")
                    recorder.finalize()
                    print("[Renderer] Recorder finalized successfully.")
                print("[Renderer] Diagnostics:", recorder.diagnostics)
            except Exception as e:
                print(f"[Renderer ERROR] Failed to finalize recorder: {e}")
        isaac_app.close()

if __name__ == "__main__":
    sys.exit(main())
