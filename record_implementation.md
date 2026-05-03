# LeRobot Dataset v3 Recording Integration Plan

## Purpose

Build a modular recording pipeline for Quest + Isaac Sim teleoperation that writes LeRobotDataset v3 episodes for VLA training. The first target is the OpenArm bimanual setup with:

- `observation.state`: current robot joint state, default 16 dimensions.
- `action`: commanded robot action, default 16 dimensions.
- `observation.images.<camera>`: multiple Isaac camera RGB feeds, for example head, left wrist, and right wrist.

The implementation must be robot-configurable, avoid adding meaningful latency to teleoperation, and support Quest controller recording controls:

- `A`: change camera view in the Isaac viewport.
- `B`: reset the Isaac scene.
- `X`: save the current episode recording.
- `Y`: discard the current episode recording.

## Source Understanding

The LeRobotDataset v3 format separates storage from API-level access:

- Tabular streams such as state, action, timestamp, frame index, task index, and episode index are stored in chunked Parquet files.
- Visual streams are stored as MP4 video shards grouped by camera key.
- Metadata in `meta/info.json`, `meta/stats.json`, `meta/tasks.jsonl` or `tasks.parquet`, and `meta/episodes/` describes schema, fps, stats, task mapping, episode lengths, and offsets into the larger Parquet/MP4 files.
- v3 intentionally stores multiple episodes inside larger data/video files, then uses metadata to reconstruct episode boundaries. This avoids the v2 problem of one Parquet/MP4 per episode.
- Recording should use the public API:
  - `LeRobotDataset.create(repo_id, fps, features, root, robot_type, use_videos=True, ...)`
  - `dataset.add_frame(frame)`
  - `dataset.save_episode()`
  - `dataset.clear_episode_buffer(delete_images=True)`
  - `dataset.finalize()`
  - optionally `dataset.push_to_hub()`
- Each `add_frame` payload must include all user-defined features plus a string `task`. Do not provide `timestamp`, `frame_index`, `episode_index`, `index`, or `task_index`; LeRobot computes those.
- Call `finalize()` before reading, pushing, or exiting. v3 uses incremental Parquet writers and pending video encoders, so missing `finalize()` can leave invalid files.

Primary references:

- Hugging Face blog post supplied by user: https://github.com/huggingface/blog/blob/main/lerobot-datasets-v3.md
- Current LeRobot v3 docs: https://huggingface.co/docs/lerobot/en/lerobot-dataset-v3
- Current LeRobotDataset source API: https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/lerobot_dataset.py
- Current DatasetWriter behavior: https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/dataset_writer.py
- Current feature validation behavior: https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/feature_utils.py

## Current Repo Architecture Relevant To Recording

Important existing files:

- `src/launch/openarm_runtime.py`
  - Main OpenArm Isaac control loop.
  - Computes `action = adapter.compute_action(session_update.targets)`.
  - Applies action via `adapter.apply_action(action)`.
  - Publishes `/joint_states`.
  - Calls `camera_manager.update(stamp=stamp)`.
  - Currently maps primary button to camera switching.

- `src/launch/panda_teleop.py`
  - Similar single-arm loop, but less factored than OpenArm runtime.

- `src/launch/controller_provider.py`
  - Subscribes to `/quest/<hand>_hand/pose` and `/quest/<hand>_hand/inputs`.
  - Maps axes and buttons into `ControllerAxes` and `ControllerButtons`.
  - Currently exposes `camera_switch_pressed` as any primary button.
  - In ROS Joy messages, button index 0 is `button_a_x` and button index 1 is `button_b_y`.
  - Because Quest button names depend on hand, the right controller maps primary/secondary to A/B and the left controller maps primary/secondary to X/Y.

- `src/isaac_backend/camera_manager.py`
  - Creates Replicator render products and RGB annotators.
  - Captures camera images every `publish_interval_frames`.
  - Publishes camera images using a bounded queue and background publisher thread.
  - Owns viewport camera switching.

- `src/robot_adapters/base.py`
  - Defines `RobotAction(joint_positions)` and the base `RobotAdapter`.
  - Adapter already abstracts current joint state, action computation, joint names, camera specs, and viewport cameras.

- `src/robot_adapters/openarm.py`
  - Gets all articulation joint names after loading.
  - OpenArm config has 7 left arm joints, 7 right arm joints, 2 left gripper joints, and 2 right gripper joints. Full articulation recording would be 18-D, while the requested 16-D dataset should use 14 arm joints plus one scalar gripper value per hand.
  - Cameras are in robot config under `cameras`.

- `config/config.yaml`
  - Central place for runtime settings.

- `config/robots/openarm.yaml`
  - Robot-specific joint and camera configuration.

## Design Goals

1. Preserve teleoperation frame rate.
   - The control loop must only do cheap snapshots and bounded nonblocking queue operations.
   - Disk writes, video encoding, stats, and LeRobot `save_episode()` must happen outside the Isaac control step.

2. Keep robot configuration modular.
   - State/action feature dimensions, joint names, camera keys, camera resolution, fps, and dataset task text must be driven by YAML and adapter helpers.
   - New robots should not require changes to the core recorder.

3. Use LeRobot v3 public APIs.
   - Do not manually create Parquet/MP4/metadata files.
   - Let `LeRobotDataset` own chunking, metadata, video encoding, stats, and task indices.

4. Keep recording semantics explicit.
   - An episode records frames continuously after calibration and while recording is enabled.
   - `X` commits the current buffered episode with `save_episode()`.
   - `Y` discards the current buffered episode with `clear_episode_buffer(delete_images=True)`.
   - `B` resets the scene and must discard any unsaved frame buffer unless config explicitly allows reset-with-buffer.
   - `A` only switches viewport camera.

5. Make failure behavior safe.
   - Recorder queue full should drop recording frames, not slow the robot.
   - Save/discard/reset/finalize actions should be logged and debounced.
   - Shutdown must call `finalize()` if the dataset was created or resumed.

## Proposed Data Schema

Default OpenArm feature schema should be 16-D, not the full 18 articulation DOFs. Record 14 arm joints plus one normalized scalar per gripper side:

- 7 left arm joints
- 1 normalized left gripper scalar
- 7 right arm joints
- 1 normalized right gripper scalar

For a robot or training setup that needs all articulation DOFs, change the YAML to `state.mode: articulation_joints` and `action.mode: articulation_joints`, which will record the full simulator vector.

```python
state_action_names = [
    "openarm_left_joint1",
    "openarm_left_joint2",
    "openarm_left_joint3",
    "openarm_left_joint4",
    "openarm_left_joint5",
    "openarm_left_joint6",
    "openarm_left_joint7",
    "left_gripper",
    "openarm_right_joint1",
    "openarm_right_joint2",
    "openarm_right_joint3",
    "openarm_right_joint4",
    "openarm_right_joint5",
    "openarm_right_joint6",
    "openarm_right_joint7",
    "right_gripper",
]

features = {
    "observation.state": {
        "dtype": "float32",
        "shape": (16,),
        "names": state_action_names,
    },
    "action": {
        "dtype": "float32",
        "shape": (16,),
        "names": state_action_names,
    },
    "observation.images.head": {
        "dtype": "video",
        "shape": (3, 360, 480),
        "names": ["channel", "height", "width"],
    },
    "observation.images.wrist_left": {
        "dtype": "video",
        "shape": (3, 360, 480),
        "names": ["channel", "height", "width"],
    },
    "observation.images.wrist_right": {
        "dtype": "video",
        "shape": (3, 360, 480),
        "names": ["channel", "height", "width"],
    },
}
```

Frame payload at runtime:

```python
frame = {
    "observation.state": np.asarray(state_vec, dtype=np.float32),
    "action": np.asarray(action_vec, dtype=np.float32),
    "observation.images.head": head_rgb_uint8,          # HWC or CHW accepted by LeRobot
    "observation.images.wrist_left": wrist_left_rgb_uint8,
    "observation.images.wrist_right": wrist_right_rgb_uint8,
    "task": current_task_string,
}
```

Do not include timestamps manually. LeRobot derives timestamp as `frame_index / fps`.

## Configuration Additions

Add a top-level `recording` section to `config/config.yaml`:

```yaml
recording:
  enabled: false
  root: "datasets"
  repo_id: "local/quest3-openarm"
  task: "Teleoperate OpenArm to complete the task"
  fps: 30
  start_after_calibration: true
  auto_start_episode: true
  use_videos: true
  streaming_encoding: true
  vcodec: "auto"
  encoder_threads: 2
  image_writer_threads: 2
  queue_size_frames: 8
  drop_when_full: true
  save_parallel_encoding: true
  finalize_on_shutdown: true
  push_to_hub_on_shutdown: false
  private_hub_repo: false
  buttons:
    switch_camera: "right_primary"   # Quest A
    reset_scene: "right_secondary"   # Quest B
    save_episode: "left_primary"     # Quest X
    discard_episode: "left_secondary" # Quest Y
  reset_policy:
    discard_unsaved_episode: true
  state:
    key: "observation.state"
    dtype: "float32"
    mode: "named_groups"
    groups:
      - "left_arm"
      - "left_gripper_scalar"
      - "right_arm"
      - "right_gripper_scalar"
  action:
    key: "action"
    dtype: "float32"
    mode: "named_groups"
    groups:
      - "left_arm"
      - "left_gripper_scalar"
      - "right_arm"
      - "right_gripper_scalar"
  cameras:
    enabled: true
    include: ["head", "wrist_left", "wrist_right"]
    feature_prefix: "observation.images"
    dtype: "video"
    resolution: [480, 360]
```

Add robot-specific recording metadata in `config/robots/openarm.yaml`:

```yaml
recording:
  robot_type: "openarm"
  joint_groups:
    left_arm:
      source: "articulation"
      joints:
        - "openarm_left_joint1"
        - "openarm_left_joint2"
        - "openarm_left_joint3"
        - "openarm_left_joint4"
        - "openarm_left_joint5"
        - "openarm_left_joint6"
        - "openarm_left_joint7"
    right_arm:
      source: "articulation"
      joints:
        - "openarm_right_joint1"
        - "openarm_right_joint2"
        - "openarm_right_joint3"
        - "openarm_right_joint4"
        - "openarm_right_joint5"
        - "openarm_right_joint6"
        - "openarm_right_joint7"
    left_gripper_scalar:
      source: "adapter"
      name: "left_gripper"
      value: "normalized_gripper"
      side: "left"
    right_gripper_scalar:
      source: "adapter"
      name: "right_gripper"
      value: "normalized_gripper"
      side: "right"
```

Reasoning:

- Main config controls recorder behavior and LeRobot storage options.
- Robot config controls how to convert simulator/adapter state into dataset vectors.
- Camera prim paths stay in the existing robot `cameras` section.

## New Package Layout

Create `src/recording/`:

```text
src/recording/
  __init__.py
  config.py
  schema.py
  buttons.py
  snapshots.py
  lerobot_recorder.py
  async_worker.py
```

### `src/recording/config.py`

Responsibilities:

- Dataclasses for `RecordingConfig`, `RecordingButtonConfig`, `RecordingResetPolicy`, `RecordingVectorConfig`, and `RecordingCameraConfig`.
- `RecordingConfig.from_mapping(values, project_root)` that resolves `root` relative to repo root.
- Validate:
  - `fps > 0`
  - `queue_size_frames >= 1`
  - camera resolution is `[width, height]`
  - button names are valid symbolic names.
- Expose `enabled` so launchers can build recorder only when requested.

Important defaults:

- `enabled=False`
- `streaming_encoding=True`
- `vcodec="auto"`
- `encoder_threads=2`
- `queue_size_frames=8`
- `drop_when_full=True`
- `finalize_on_shutdown=True`

### `src/recording/buttons.py`

Responsibilities:

- Convert hand-relative Quest buttons into named semantic events.
- Add edge detection to avoid repeated save/discard/reset while a button is held.

Button symbols:

- `right_primary`: A
- `right_secondary`: B
- `left_primary`: X
- `left_secondary`: Y
- `any_primary`
- `any_secondary`

Class sketch:

```python
@dataclass
class RecordingButtonEvents:
    switch_camera: bool = False
    reset_scene: bool = False
    save_episode: bool = False
    discard_episode: bool = False

class ButtonEdgeMapper:
    def __init__(self, config: RecordingButtonConfig): ...
    def update(self, latest_states: dict[str, ControllerState | None]) -> RecordingButtonEvents: ...
```

Implementation detail:

- Read buttons from `ControllerState.buttons.primary` and `ControllerState.buttons.secondary`.
- Right controller primary/secondary are A/B.
- Left controller primary/secondary are X/Y.
- Store previous boolean state for each semantic action; emit only rising edges.

Also update `src/launch/controller_provider.py`:

- Add:
  - `button_pressed(symbol: str) -> bool`
  - `latest_buttons() -> dict[str, ControllerButtons]`
  - optional properties `right_primary_pressed`, `right_secondary_pressed`, `left_primary_pressed`, `left_secondary_pressed`.
- Keep `camera_switch_pressed` for backward compatibility, but update OpenArm to use `ButtonEdgeMapper`.

### `src/recording/schema.py`

Responsibilities:

- Build LeRobot `features` from adapter + runtime config.
- Build vector extractors for state/action.
- Avoid hardcoding OpenArm.

Dataclasses:

```python
@dataclass(frozen=True)
class VectorSpec:
    key: str
    names: tuple[str, ...]
    shape: tuple[int,]
    dtype: str

@dataclass(frozen=True)
class CameraFeatureSpec:
    camera_name: str
    feature_key: str
    shape: tuple[int, int, int]  # C, H, W
    dtype: str = "video"

@dataclass(frozen=True)
class RecordingSchema:
    features: dict[str, dict]
    state_spec: VectorSpec
    action_spec: VectorSpec
    camera_specs: tuple[CameraFeatureSpec, ...]
```

Function sketch:

```python
def build_recording_schema(adapter, robot_config: dict, recording_config: RecordingConfig) -> RecordingSchema:
    state_names = resolve_vector_names(adapter, robot_config, recording_config.state)
    action_names = resolve_vector_names(adapter, robot_config, recording_config.action)
    camera_features = resolve_camera_features(adapter.get_camera_specs(), recording_config.cameras)
    return RecordingSchema(...)
```

Feature shape rules:

- State/action:
  - shape `(len(names),)`
  - dtype from config, usually `float32`.
- Video:
  - LeRobot expects image/video shape `(C, H, W)`.
  - The repo's camera config stores resolution as `[width, height]`.
  - Feature shape must be `(3, height, width)`.

### Adapter Extensions

Extend `RobotAdapter` in `src/robot_adapters/base.py` with optional recording hooks:

```python
def get_recording_vector(
    self,
    *,
    vector_config: dict,
    current_joint_positions,
    commanded_action,
    teleop_targets=None,
) -> tuple[list[str], np.ndarray]:
    raise NotImplementedError

def get_recording_robot_type(self) -> str:
    return self.__class__.__name__.replace("Adapter", "").lower()
```

To avoid breaking existing adapters, provide a default helper that can record full articulation:

```python
def get_articulation_vector_by_joint_names(self, joint_names, joint_positions) -> np.ndarray:
    name_to_index = {name: idx for idx, name in enumerate(self.get_joint_names())}
    return np.asarray([joint_positions[name_to_index[name]] for name in joint_names], dtype=np.float32)
```

OpenArm-specific implementation:

- For `left_arm` and `right_arm`, read named joints from `current_joint_positions` for state and from `commanded_action.joint_positions` for action.
- For `left_gripper_scalar` and `right_gripper_scalar`, convert adapter gripper values to normalized scalar.
  - For state: use actual articulation gripper joint positions and normalize from open/closed range.
  - For action: use commanded target gripper positions and normalize from open/closed range.
- Normalize gripper with:

```python
def normalize_gripper(raw, open_position, closed_position):
    denom = closed_position - open_position
    if abs(denom) < 1e-9:
        return 0.0
    return np.clip((raw - open_position) / denom, 0.0, 1.0)
```

This maps open to 0 and closed to 1 even if closed is numerically smaller than open.

Panda-specific implementation can be added after OpenArm:

- Default 8 or 9 dimensions depending on config.
- `arm` joints plus either one gripper scalar or both finger joints.

### `src/recording/snapshots.py`

Responsibilities:

- Immutable payloads passed from the Isaac control loop to the recorder worker.
- Keep frame snapshots small and explicit.

Dataclasses:

```python
@dataclass
class CameraFrameSnapshot:
    feature_key: str
    image_rgb: np.ndarray

@dataclass
class RecordingFrameSnapshot:
    state: np.ndarray
    action: np.ndarray
    cameras: dict[str, np.ndarray]
    task: str
    monotonic_time_s: float
    sequence: int | None = None
```

### `src/recording/lerobot_recorder.py`

Responsibilities:

- Own the `LeRobotDataset` object and translate snapshots to `dataset.add_frame`.
- Provide public methods for launch loops:
  - `start()`
  - `enqueue_frame(snapshot) -> bool`
  - `save_episode_async()`
  - `discard_episode_async()`
  - `finalize()`
  - `close()`
  - `diagnostics`

Class sketch:

```python
class LeRobotEpisodeRecorder:
    def __init__(self, config: RecordingConfig, schema: RecordingSchema, robot_type: str):
        self.dataset = LeRobotDataset.create(
            repo_id=config.repo_id,
            root=config.root,
            fps=config.fps,
            features=schema.features,
            robot_type=robot_type,
            use_videos=config.use_videos,
            image_writer_threads=config.image_writer_threads,
            streaming_encoding=config.streaming_encoding,
            vcodec=config.vcodec,
            encoder_threads=config.encoder_threads,
        )
```

Important:

- Import LeRobot lazily inside this class so tests that do not install LeRobot can still import other modules.
- If `recording.enabled=true` and LeRobot import fails, raise a clear runtime error with install instructions.
- `enqueue_frame` must be nonblocking. If full and `drop_when_full=true`, increment dropped counter and return `False`.
- `save_episode_async` should enqueue a command to the worker, not run `dataset.save_episode()` in the Isaac loop.
- `discard_episode_async` should enqueue a command that calls `dataset.clear_episode_buffer(delete_images=True)`.
- `finalize` should drain/stop the worker, then call `dataset.finalize()`.

### `src/recording/async_worker.py`

Responsibilities:

- Single background writer thread.
- Process frame snapshots and commands in order.
- Avoid concurrent calls into a single `LeRobotDataset`.

Use one `queue.Queue(maxsize=config.queue_size_frames)` for frames and a small command queue or a unified queue:

```python
@dataclass
class RecorderCommand:
    kind: Literal["frame", "save", "discard", "finalize", "stop"]
    payload: object | None = None
```

Unified queue is simpler because save/discard order matters relative to frames.

Worker loop:

```python
while running:
    cmd = queue.get(timeout=0.1)
    if cmd.kind == "frame":
        dataset.add_frame(make_frame_dict(cmd.payload))
    elif cmd.kind == "save":
        if dataset.has_pending_frames():
            dataset.save_episode(parallel_encoding=config.save_parallel_encoding)
            diagnostics.saved_episodes += 1
        else:
            diagnostics.empty_save_requests += 1
    elif cmd.kind == "discard":
        dataset.clear_episode_buffer(delete_images=True)
        diagnostics.discarded_episodes += 1
    elif cmd.kind == "finalize":
        dataset.finalize()
    elif cmd.kind == "stop":
        break
```

Queue policy:

- Frame enqueue should use `put_nowait`.
- Save/discard/finalize commands should either use a separate unbounded command queue or force space by dropping old frame commands. Do not allow a full frame queue to block a save/discard button.
- Recommended: two queues. Worker prioritizes command queue first, then frame queue. For save/discard, first call `frame_queue.join()` with a short bounded wait or drain queued frames synchronously before saving. This ensures recently captured frames are included but avoids indefinite blocking.

Preferred concrete design:

- `frame_queue`: bounded.
- `command_queue`: unbounded but low volume.
- Worker checks command queue first.
- For `save`, process all currently queued frames before calling `save_episode()`.
- For `discard`, clear `frame_queue` first, then call `clear_episode_buffer()`.

## Camera Capture Changes

Current `CameraManager.update()` captures images and pushes them to the ROS publisher queue, but it does not return frames to callers.

Modify `src/isaac_backend/camera_manager.py` in a backward-compatible way:

```python
def update(self, stamp=None, return_frames: bool = False) -> dict[str, np.ndarray] | None:
    captured = {}
    ...
    image_rgb = self._coerce_rgb(camera_name, data)
    ...
    if return_frames:
        captured[camera_name] = image_rgb
    if self.camera_publishers is not None:
        self._camera_queue.put_nowait((camera_name, image_rgb, stamp))
    ...
    return captured if return_frames else None
```

Optimization:

- Avoid copying image data twice. `_coerce_rgb` already creates a contiguous uint8 copy. Reuse that same array for ROS publishing and recording.
- Only return frames on frames that pass `publish_interval_frames`.
- When recording fps differs from camera publish interval, add separate `record_interval_frames` to camera or recording config. First implementation can require `recording.fps == simulation_control_fps / publish_interval_frames` or simply record when frames are available.

Potential issue:

- If `CameraManager` returns no frame on skipped intervals, the recorder should skip that control step. LeRobot expects every frame to include all camera keys, so do not call `add_frame` unless all configured camera frames are present.

Recommended camera scheduling for v1:

- Record exactly at camera capture cadence. If `publish_interval_frames=2` and Isaac runs at about 60 Hz, dataset fps should be 30.
- Validate this in config and document it.

Future improvement:

- Add `record_interval_frames` separate from `publish_interval_frames` so ROS camera publishing and dataset recording can run at different rates.

## Control Loop Integration

Modify `src/launch/openarm_runtime.py`.

New imports:

```python
from src.recording import (
    ButtonEdgeMapper,
    LeRobotEpisodeRecorder,
    RecordingConfig,
    build_recording_schema,
    make_frame_snapshot,
)
```

`run_openarm_runtime` signature additions:

```python
def run_openarm_runtime(..., recording_config: dict | None = None) -> int:
```

Setup after adapter joint mappings and camera manager start:

```python
recording = RecordingConfig.from_mapping(recording_config, project_root=...)
recorder = None
button_mapper = ButtonEdgeMapper(recording.buttons)
if recording.enabled:
    schema = build_recording_schema(adapter, adapter.config, recording)
    recorder = LeRobotEpisodeRecorder(
        config=recording,
        schema=schema,
        robot_type=adapter.get_recording_robot_type(),
    ).start()
```

Update ready print:

- Print recording enabled/disabled.
- Print dataset root and repo id if enabled.
- Print button mapping.

Inside `_run_control_loop`:

1. Spin ROS.
2. Build `session_update`.
3. Compute button edge events with latest controller states.
4. Handle camera switch if `events.switch_camera`.
5. Handle reset if `events.reset_scene`.
6. Skip recording while not calibrated/ready.
7. Compute current state before action.
8. Compute and apply action.
9. Capture camera frames.
10. Enqueue recorder snapshot if enabled and all data present.
11. Step Isaac.

Pseudo-code:

```python
events = button_mapper.update(controller_provider.latest())

if events.switch_camera:
    camera_manager.switch_viewport_camera_next()

if events.discard_episode and recorder:
    recorder.discard_episode_async(reason="quest_y")

if events.save_episode and recorder:
    recorder.save_episode_async(reason="quest_x")

if events.reset_scene:
    if recorder and recording.reset_policy.discard_unsaved_episode:
        recorder.discard_episode_async(reason="scene_reset")
    reset_scene_for_recording(...)
    continue
```

Important ordering:

- Process `save_episode` and `discard_episode` edge events before adding the current frame, otherwise the button-press frame will go into the next episode or an episode being discarded.
- For reset, discard first, then reset Isaac and teleop session state.

Frame snapshot pseudo-code:

```python
current_positions = adapter.get_current_joint_positions()
action = adapter.compute_action(session_update.targets)
adapter.apply_action(action)
teleop_session.mark_isaac_apply()
stamp = controller_provider.get_clock().now().to_msg()
joint_state_publisher.publish(dof_names, action.joint_positions, stamp=stamp)

camera_frames = camera_manager.update(stamp=stamp, return_frames=recording.enabled)

if recorder and camera_frames and recording_should_capture_this_tick:
    post_apply_positions = adapter.get_current_joint_positions()
    state_vec = adapter.get_recording_vector(
        vector_config=recording.state,
        current_joint_positions=post_apply_positions or current_positions,
        commanded_action=action,
        teleop_targets=session_update.targets,
    )
    action_vec = adapter.get_recording_vector(
        vector_config=recording.action,
        current_joint_positions=current_positions,
        commanded_action=action,
        teleop_targets=session_update.targets,
    )
    recorder.enqueue_frame(
        RecordingFrameSnapshot(
            state=state_vec,
            action=action_vec,
            cameras=filter_and_key_cameras(camera_frames, schema),
            task=recording.task,
            monotonic_time_s=time.monotonic(),
            sequence=max_controller_sequence(controller_provider.latest()),
        )
    )
```

State timing decision:

- For VLA imitation learning, `observation.state[t]` should represent the robot state observed when `action[t]` was commanded.
- Recommended first implementation: sample `state` immediately before `compute_action` and store it with the action that is then applied.
- If post-apply state is preferred later, make this configurable as `recording.state.sample_timing: "pre_action" | "post_action"`.

## Scene Reset Implementation

`B` should reset the scene in the viewport.

Add helper in `openarm_runtime.py`:

```python
def _reset_scene(
    isaac_app: IsaacApp,
    adapter: OpenArmAdapter,
    teleop_session: BimanualTeleopSession,
    controller_provider,
) -> None:
    isaac_app.reset_world()
    adapter.reset_runtime_state()
    teleop_session.reset_calibration_or_filters(...)
```

Required adapter/session additions:

- `OpenArmAdapter.reset_runtime_state()`
  - Reset `left_runtime.last_arm_positions`.
  - Reset `right_runtime.last_arm_positions`.
  - Reset smoothed grippers to open.
  - Clear diagnostic counters only if config says so; default keep counters.

- `BimanualTeleopSession.reset()`
  - Clear calibration and filters so the user recalibrates after scene reset.
  - Or add `reset_filters_keep_calibration()` if you want reset not to force recalibration.

Recommended behavior:

- Reset world.
- Clear/refresh adapter runtime state.
- Clear teleop calibration and require controllers steady again.
- Discard unsaved recording buffer.

Why:

- After world reset, old controller-to-robot calibration and pending frames may no longer match the scene state.

## Dataset Lifecycle

Startup:

1. Load config.
2. Create adapter and initialize joint mappings.
3. Create camera manager.
4. If recording enabled:
   - Build schema.
   - Create local dataset root.
   - Create `LeRobotDataset`.
   - Start recorder worker.

During control:

- Add frames only when:
  - teleop session is ready,
  - current joint positions are valid,
  - action was computed,
  - all configured camera frames are available,
  - no save/discard/reset command is currently being applied.

Save:

- `X` queues `save_episode()`.
- Worker drains current frame queue, calls `dataset.save_episode(parallel_encoding=True)`, updates diagnostics.
- Recording continues into a new episode automatically because LeRobot clears the buffer after save.

Discard:

- `Y` clears queued frames and calls `dataset.clear_episode_buffer(delete_images=True)`.
- Recording continues into a fresh episode.

Shutdown:

- Stop accepting frames.
- Ask worker to finish queued frames or discard based on config:
  - Recommended default: do not auto-save incomplete episode.
  - If `recording.auto_save_on_shutdown=true`, save pending frames.
  - Else call clear buffer.
- Call `dataset.finalize()`.
- Optional `push_to_hub()` only if configured.

## Performance Plan

Control-loop overhead should be limited to:

- Reading current joint positions.
- Copying/normalizing a small state/action vector.
- Getting camera frames already captured by `CameraManager`.
- `queue.put_nowait` of a snapshot.

Avoid in the control loop:

- `dataset.add_frame`.
- `dataset.save_episode`.
- image encoding,
- Parquet writes,
- video concatenation,
- Hub upload.

Recommended LeRobot writer settings:

- `streaming_encoding: true`
  - Encodes videos while recording instead of writing PNGs first, making save faster.
- `vcodec: "auto"`
  - Lets LeRobot choose a hardware-supported encoder when available.
- `encoder_threads: 2`
  - Caps encoder CPU usage.
- `queue_size_frames: 8`
  - Small enough to avoid memory growth, large enough to absorb short writer hiccups.

Camera memory:

- 480x360x3 uint8 is about 0.52 MB per camera.
- Three cameras are about 1.56 MB per recorded frame.
- A frame queue of 8 is about 12.5 MB plus Python overhead.

Backpressure:

- If frame queue fills, drop recording frames and increment diagnostics.
- Never block Isaac stepping because a dataset frame could not be recorded.
- Log dropped recording frames at a low rate, for example every 5 seconds.

Diagnostics:

Track:

- `frames_enqueued`
- `frames_written`
- `frames_dropped_queue_full`
- `save_requests`
- `saved_episodes`
- `discard_requests`
- `discarded_episodes`
- `empty_save_requests`
- `worker_errors`
- `last_error`
- `queue_depth`

Print in session statistics alongside existing IK and camera diagnostics.

## CLI Changes

Update `src/launch/openarm_teleop.py`:

New arguments:

```python
parser.add_argument("--record", action="store_true", help="Enable LeRobot dataset recording")
parser.add_argument("--dataset-root", help="Override recording.root")
parser.add_argument("--dataset-repo-id", help="Override recording.repo_id")
parser.add_argument("--task", help="Override recording.task")
parser.add_argument("--recording-fps", type=int, help="Override recording.fps")
```

In `resolve_runtime_settings`, return `recording_config` as an additional item. Merge CLI overrides into `runtime.main["recording"]`.

Update `run_openarm_runtime(...)` call to pass `recording_config`.

Later update `panda_teleop.py` similarly, but implement OpenArm first because it is already factored through `run_openarm_runtime`.

## Dependency Changes

Update `requirements.txt`:

```text
lerobot>=0.4.0
```

Potentially do not pin LeRobot too tightly because docs currently show v0.5.1 available. The plan assumes the v3 public API from LeRobot main:

- `from lerobot.datasets import LeRobotDataset`
- fallback: `from lerobot.datasets.lerobot_dataset import LeRobotDataset`

If Isaac's Python environment cannot install LeRobot directly, document installing it inside the Python interpreter used by Isaac Sim.

## Tests

Unit tests should avoid importing Isaac and should not require LeRobot unless specifically testing recorder integration.

Add `tests/test_recording_config.py`:

- `RecordingConfig.from_mapping` resolves root path.
- Defaults are sane.
- Invalid fps raises.
- Button names validate.

Add `tests/test_recording_buttons.py`:

- Right primary emits `switch_camera` for A.
- Right secondary emits `reset_scene` for B.
- Left primary emits `save_episode` for X.
- Left secondary emits `discard_episode` for Y.
- Holding button emits only one edge.

Add `tests/test_recording_schema.py`:

- Build OpenArm schema from `config/robots/openarm.yaml`.
- Assert state/action shapes are `(16,)` when using gripper scalar config.
- Assert camera feature keys:
  - `observation.images.head`
  - `observation.images.wrist_left`
  - `observation.images.wrist_right`
- Assert video shapes are `(3, 360, 480)` for resolution `[480, 360]`.

Add `tests/test_openarm_recording_vectors.py`:

- Use a fake adapter or OpenArmAdapter with fake `dof_names`.
- Verify named-group vector extraction order.
- Verify gripper normalization maps open to 0 and closed to 1.
- Verify action vector uses commanded action values, not current state.

Add optional `tests/test_lerobot_recorder_smoke.py`:

- Mark/skips if LeRobot is not installed.
- Create a temporary dataset with one tiny camera frame, for example 64x48.
- Add 2 frames, save episode, finalize.
- Reload `LeRobotDataset(root=tmpdir, repo_id=...)` and assert length is 2.

Update `tests/test_launch_phase9.py`:

- Assert `--record` enables recording config.
- Assert CLI overrides root/repo/task/fps.

## Implementation Order

1. Add config dataclasses and defaults.
2. Add button edge mapper and `controller_provider` accessors.
3. Add schema builder and OpenArm recording vector extraction.
4. Modify `CameraManager.update` to optionally return captured RGB frames.
5. Add async LeRobot recorder wrapper with lazy LeRobot import.
6. Wire recorder into `openarm_teleop.py` and `openarm_runtime.py`.
7. Add scene reset handling for B.
8. Add tests for config/buttons/schema/vector extraction.
9. Add LeRobot smoke test guarded by import availability.
10. Run unit tests.
11. Run an Isaac manual test with recording enabled.

## Manual Validation Checklist

Before using the dataset for VLA training:

1. Start bridge and OpenArm teleop with recording enabled:

```bash
./scripts/run_openarm_teleop.sh --record --dataset-repo-id local/quest3-openarm-test --task "Pick up the object"
```

If the shell script does not pass arguments through, update it to forward `"$@"`.

2. Wait for calibration.
3. Move robot for several seconds.
4. Press `A`; viewport camera should cycle.
5. Press `X`; terminal should report one episode saved.
6. Move robot again.
7. Press `Y`; terminal should report current episode discarded.
8. Move robot again.
9. Press `B`; scene should reset and recording buffer should be discarded.
10. Stop process cleanly; recorder must call `finalize()`.
11. Load local dataset:

```python
from lerobot.datasets import LeRobotDataset
dataset = LeRobotDataset("local/quest3-openarm-test", root="datasets/local/quest3-openarm-test")
print(dataset)
print(dataset[0].keys())
print(dataset[0]["observation.state"].shape)
print(dataset[0]["action"].shape)
print(dataset[0]["observation.images.head"].shape)
```

Expected:

- `observation.state` shape is `[16]`.
- `action` shape is `[16]`.
- Each image is `[C, H, W]`.
- Dataset has only saved episodes, not discarded/reset buffers.

## Edge Cases To Handle

- Recording enabled but no cameras configured:
  - Allow state/action-only dataset if `recording.cameras.enabled=false`.
  - If cameras enabled but missing, raise clear config error.

- Camera frame missing for one configured camera:
  - Skip that dataset frame and increment `incomplete_camera_frame_drops`.
  - Do not add partial frames; LeRobot requires all configured features per frame.

- Queue full:
  - Drop frame, log rate-limited warning.

- Save with no pending frames:
  - Do not crash. Increment `empty_save_requests`.

- Discard with no pending frames:
  - Do not crash.

- Exception in worker:
  - Store exception in diagnostics.
  - Log with traceback.
  - Disable further enqueue or keep dropping frames until shutdown.

- User exits with Ctrl+C:
  - Stop worker and finalize dataset.

- LeRobot unavailable:
  - Only error if recording is enabled.
  - Non-recording teleop should continue to work.

## Notes For GPT 5.4 Implementer

- Keep imports of Isaac-only modules inside functions/classes that already require Isaac. Unit tests should still import most modules without Isaac installed.
- Use `apply_patch`-friendly small edits. Do not refactor unrelated teleop logic.
- Preserve existing camera publishing behavior. Returning camera frames from `CameraManager.update` must be additive.
- Preserve existing `camera_switch_pressed` property until all launchers are migrated.
- Use LeRobot public API only. Do not write Parquet/MP4 metadata manually.
- Use `np.float32` for state/action arrays because LeRobot validates exact dtype and shape.
- Use channel-last `HWC` uint8 camera arrays from `CameraManager`; LeRobot accepts HWC or CHW as long as the feature shape is `(C, H, W)`.
- Never call `dataset.save_episode()` or `dataset.finalize()` in the Isaac frame loop.
- Prefer OpenArm integration first. Panda can be wired after the OpenArm path is tested.

## Acceptance Criteria

- Teleoperation still runs with `recording.enabled=false`.
- With recording enabled, pressing A/B/X/Y performs the requested action with rising-edge semantics.
- Saved dataset follows LeRobot v3 layout with `meta/`, `data/`, and `videos/`.
- Saved OpenArm episodes contain 16-D `observation.state`, 16-D `action`, and all configured camera streams.
- Discarded episodes do not appear when reloading the dataset.
- Scene reset discards unsaved recording frames and does not corrupt the dataset.
- Unit tests pass without Isaac installed.
- A guarded LeRobot smoke test passes when LeRobot is installed.
- Shutdown calls `finalize()` and produces a loadable dataset.
