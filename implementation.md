# Implementation Plan: Production-Grade Quest 3 Teleoperation Framework

This document is the implementation guide for evolving this repository from a working OpenArm-specific prototype into a modular, efficient teleoperation framework that can support multiple robot embodiments in Isaac Sim 5.1.0, including the current OpenArm bimanual model.

The current system works, but the main application logic is concentrated in `src/isaac_openarm_teleop.py`. The goal is to split transport, teleoperation semantics, robot embodiment details, Isaac Sim control, and camera/recording into separate modules while preserving the existing working workflow.

Use this document as the primary roadmap for code changes.

## Current Architecture Summary

Current runtime path:

```text
Quest 3 Browser
  -> web/webxr_streamer.html
  -> WSS JSON packets
  -> src/webxr_ros_bridge.py
  -> ROS 2 topics:
       /quest/left_hand/pose
       /quest/right_hand/pose
       /quest/left_hand/inputs
       /quest/right_hand/inputs
  -> src/isaac_openarm_teleop.py
  -> Isaac Sim OpenArm articulation
  -> /joint_states and /camera/*/image_raw
```

Important current files:

- `web/webxr_streamer.html`: Quest browser WebXR app.
- `web/https_server.py`: HTTPS static server required for WebXR secure context.
- `src/webxr_ros_bridge.py`: WebSocket server that receives WebXR JSON and publishes ROS 2 topics.
- `src/isaac_openarm_teleop.py`: Current main OpenArm teleoperation script.
- `src/isaac_panda_teleop.py`: Current single-arm Panda teleoperation script.
- `scripts/run_wireless.sh`: Starts HTTPS server and WebXR ROS bridge.
- `scripts/run_openarm_teleop.sh`: Starts Isaac Sim Python for OpenArm.
- `scripts/run_panda_teleop.sh`: Starts Isaac Sim Python for Panda.
- `config/config.yaml`: Existing central config, currently underused.
- `robot_configs/openarm_config/`: OpenArm USD, URDF, and Lula descriptors.

Known mismatches and technical debt:

- Docs mention WebSocket port `9090`, but active code uses `9999`.
- Isaac Sim path exists in `config/config.yaml`, but launch scripts hardcode `/home/saurabh/isaac_sim`.
- `src/isaac_openarm_teleop.py` mixes Isaac startup, ROS subscriptions, calibration, transforms, IK, safety, gripper logic, camera switching, camera publishing, and recording topics.
- WebXR packets do not include sequence numbers or explicit sample timestamps suitable for latency/jitter diagnostics.
- Quaternion smoothing is linear interpolation plus normalization, not true SLERP.
- OpenArm-specific robot facts are hardcoded in Python.
- Camera publishing silently swallows exceptions and shares main process budget with control.
- There is no replay workflow for tuning teleoperation without wearing the Quest.

## Target Architecture

Target runtime architecture:

```text
Quest WebXR Client
  -> Transport Ingress
      - WebSocket/WSS receiver
      - sequence number validation
      - timestamp handling
      - packet loss/jitter metrics
  -> Teleop Core
      - controller state model
      - calibration
      - frame transforms
      - filtering
      - retargeting
      - safety checks
  -> Robot Adapter
      - OpenArm adapter
      - Panda adapter
      - future embodiment adapters
  -> Isaac Backend
      - simulation app lifecycle
      - world/stage loading
      - articulation control
      - IK invocation
      - camera publishing
      - ROS output publishing
```

Target source layout:

```text
src/
  quest_ingress/
    __init__.py
    webxr_bridge.py
    message_types.py
    metrics.py

  teleop_core/
    __init__.py
    calibration.py
    controller_state.py
    filters.py
    frame_transforms.py
    retargeting.py
    safety.py
    session.py

  robot_adapters/
    __init__.py
    base.py
    openarm.py
    panda.py

  isaac_backend/
    __init__.py
    app.py
    articulation_controller.py
    camera_manager.py
    ros_publishers.py

  launch/
    openarm_teleop.py
    panda_teleop.py
    webxr_bridge.py

config/
  config.yaml
  robots/
    openarm.yaml
    panda.yaml
```

The existing scripts can remain, but they should call the new launch modules.

## Implementation Principles

Preserve a working path at every phase. Do not attempt a large rewrite in one commit.

Design each module so it can be unit-tested outside Isaac Sim where possible. Isaac Sim-specific imports should stay inside `isaac_backend` and robot adapter methods that genuinely require Isaac.

Prefer configuration over hardcoded robot details. Robot embodiment specifics should live in `config/robots/*.yaml` plus small adapter classes.

Keep current ROS topic names stable initially:

- `/quest/left_hand/pose`
- `/quest/right_hand/pose`
- `/quest/left_hand/inputs`
- `/quest/right_hand/inputs`
- `/joint_states`
- `/camera/head/image_raw`
- `/camera/wrist_left/image_raw`
- `/camera/wrist_right/image_raw`

After the refactor is stable, new richer ROS messages can be added without breaking the current interface.

## Phase 1: Stabilize Current Workflow

Goal: fix configuration drift, add observability, and reduce obvious safety/performance risks without changing user workflow.

### 1.1 Unify Ports and Paths

Update `config/config.yaml` to be the source of truth:

```yaml
paths:
  isaac_sim: "/home/saurabh/isaac_sim"
  openarm:
    usd: "robot_configs/openarm_config/openarm_bimanual/openarm_bimanual_env.usd"
    urdf: "robot_configs/openarm_config/openarm_bimanual_stl.urdf"
    left_arm_config: "robot_configs/openarm_config/left_arm"
    right_arm_config: "robot_configs/openarm_config/right_arm"
  panda:
    usd: "environment.usd"
  certs:
    cert: "certs/cert.pem"
    key: "certs/key.pem"

server:
  websocket_port: 9999
  https_port: 8000
  host: "0.0.0.0"
```

Required changes:

- Make `scripts/run_wireless.sh` read `server.websocket_port`, `server.https_port`, and cert paths from config.
- Make `scripts/run_openarm_teleop.sh` and `scripts/run_panda_teleop.sh` read `paths.isaac_sim` from config or support `ISAAC_SIM_PATH` env override.
- Update README and docs so all references use `9999` unless config says otherwise.
- Update `web/webxr_streamer.html` so the default port can be injected or kept in sync. Minimal first step: update hardcoded default to match config.

Acceptance criteria:

- `./scripts/run_wireless.sh` prints the same port that the web UI defaults to.
- `./scripts/run_openarm_teleop.sh` does not contain a hardcoded Isaac path except as a fallback.
- Searching `README.md docs src web scripts config` for `9090` finds no stale user-facing instructions unless deliberately listed as historical.

### 1.2 Add Transport Metadata

Enhance WebXR packets in `web/webxr_streamer.html`.

Add top-level fields:

```json
{
  "schema_version": 1,
  "sequence": 1234,
  "timestamp": 123456.7,
  "client_epoch_ms": 1710000000000,
  "controllers": {}
}
```

Where:

- `sequence`: monotonically increasing integer per browser session.
- `timestamp`: XR frame timestamp, as currently used.
- `client_epoch_ms`: `Date.now()` when packet is sent.

In `src/webxr_ros_bridge.py`, track:

- last sequence number
- dropped packet count
- packet rate
- average packet size
- receive interval jitter
- last packet age

Log a compact metrics line every 2-5 seconds.

Acceptance criteria:

- Bridge logs packet rate and dropped packet count.
- Invalid or out-of-order sequence numbers do not crash the bridge.
- Existing ROS topic publishing remains unchanged.

### 1.3 Add Deadman Timeout

Add a deadman timeout to OpenArm control.

Behavior:

- Track last received pose time for each arm.
- If either arm has no fresh pose for `deadman_timeout_ms`, hold last valid IK target.
- If timeout exceeds a larger threshold, optionally open grippers and stop updating arm targets.

Suggested config:

```yaml
teleop:
  deadman_timeout_ms: 250
  hard_timeout_ms: 1000
```

Acceptance criteria:

- Losing Quest/WebSocket connection does not cause target jumps.
- Terminal clearly reports stale controller data.

### 1.4 Replace Linear Quaternion Smoothing

Current OpenArm orientation smoothing linearly blends quaternion components and normalizes. Replace this with proper SLERP in a reusable helper.

Add to `src/teleop_core/filters.py` later, or minimally add a local helper first:

```python
def slerp_quat_wxyz(q_current, q_target, alpha):
    ...
```

Convention:

- Isaac target orientation uses `wxyz`.
- SciPy `Rotation.from_quat` expects `xyzw`.
- Keep conversion explicit.

Acceptance criteria:

- Orientation smoothing works for both arms.
- No sudden flips when controller orientation crosses quaternion sign boundaries.

## Phase 2: Extract Teleop Core Modules

Goal: move pure teleoperation logic out of Isaac scripts.

### 2.1 Controller State Model

Create `src/teleop_core/controller_state.py`.

Required dataclasses:

```python
from dataclasses import dataclass
from typing import Optional
import numpy as np

@dataclass
class ControllerButtons:
    primary: bool = False      # A on right, X on left
    secondary: bool = False    # B on right, Y on left
    menu: bool = False
    stick_click: bool = False

@dataclass
class ControllerAxes:
    trigger: float = 0.0
    squeeze: float = 0.0
    thumbstick_x: float = 0.0
    thumbstick_y: float = 0.0

@dataclass
class ControllerPose:
    position_xyz: np.ndarray
    orientation_xyzw: np.ndarray
    valid: bool = True

@dataclass
class ControllerState:
    hand: str
    sequence: int
    source_timestamp: float
    receive_time_s: float
    pose: Optional[ControllerPose]
    axes: ControllerAxes
    buttons: ControllerButtons
```

Use this internally instead of passing raw ROS messages everywhere.

Acceptance criteria:

- WebXR bridge or ROS subscriber layer can convert incoming data into `ControllerState`.
- Unit tests can construct `ControllerState` without ROS or Isaac.

### 2.2 Calibration Module

Create `src/teleop_core/calibration.py`.

Responsibilities:

- collect N samples per hand
- compute reference pose
- expose calibration status
- support recalibration later

Suggested API:

```python
class HandCalibration:
    def __init__(self, samples_required: int):
        ...

    def add_sample(self, position_xyz: np.ndarray) -> bool:
        """Returns True once calibration is complete."""

    @property
    def reference_position(self) -> np.ndarray | None:
        ...

class BimanualCalibration:
    def update(self, left: ControllerState | None, right: ControllerState | None) -> None:
        ...
```

Acceptance criteria:

- OpenArm calibration behavior remains the same from the user's perspective.
- Calibration code no longer lives inside the ROS callback.

### 2.3 Frame Transform Module

Create `src/teleop_core/frame_transforms.py`.

Responsibilities:

- transform Quest/WebXR coordinates into robot coordinates
- transform controller orientation into robot target orientation
- keep coordinate conventions explicit

Suggested API:

```python
class FrameTransform:
    def __init__(self, matrix_vr_to_robot: np.ndarray, tool_rotation_correction: np.ndarray):
        ...

    def position_offset_to_robot(self, xr_offset_xyz: np.ndarray) -> np.ndarray:
        ...

    def orientation_xyzw_to_robot_wxyz(self, xr_quat_xyzw: np.ndarray) -> np.ndarray:
        ...
```

Move the current matrix:

```python
[[0, 0, -1],
 [-1, 0, 0],
 [0, 1, 0]]
```

and 180-degree x-axis flip into this module/config.

Acceptance criteria:

- Existing OpenArm movement direction is unchanged.
- Transform code is independent of ROS and Isaac.

### 2.4 Filters Module

Create `src/teleop_core/filters.py`.

Implement:

- exponential moving average for position
- SLERP for quaternion orientation
- optional One Euro filter for position
- velocity limiter
- acceleration limiter, if time permits

Suggested first implementation:

```python
class PositionEMA:
    def __init__(self, alpha: float):
        ...

    def update(self, target: np.ndarray) -> np.ndarray:
        ...

class OrientationSlerp:
    def __init__(self, alpha: float):
        ...

    def update(self, target_wxyz: np.ndarray) -> np.ndarray:
        ...
```

Acceptance criteria:

- OpenArm uses these classes instead of inline smoothing.
- No Isaac imports in this module.

### 2.5 Safety Module

Create `src/teleop_core/safety.py`.

Responsibilities:

- detect stale controller state
- clamp targets to workspace if configured
- reject large jumps
- limit velocity
- hold last valid target on invalid input

Suggested data:

```python
@dataclass
class WorkspaceBounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
```

For OpenArm, current behavior intentionally does not clip workspace. Preserve this by making workspace bounds optional.

Acceptance criteria:

- Safety is a separate step before IK.
- OpenArm default behavior remains close to current behavior unless safety config is enabled.

## Phase 3: Robot Adapter Abstraction

Goal: make OpenArm and Panda robot-specific details pluggable.

### 3.1 Base Adapter

Create `src/robot_adapters/base.py`.

Suggested interfaces:

```python
from abc import ABC, abstractmethod

class RobotAdapter(ABC):
    @abstractmethod
    def load(self, world, stage):
        """Load or attach to robot articulation."""

    @abstractmethod
    def initialize_ik(self):
        """Initialize IK solvers."""

    @abstractmethod
    def get_current_joint_positions(self):
        ...

    @abstractmethod
    def compute_action(self, teleop_targets):
        """Return joint target vector or action object."""

    @abstractmethod
    def apply_action(self, action):
        ...

    @abstractmethod
    def get_joint_names(self) -> list[str]:
        ...

    def get_camera_specs(self) -> dict:
        return {}
```

Define teleop target types in `teleop_core/retargeting.py`, not in the adapter.

### 3.2 OpenArm Adapter

Create `src/robot_adapters/openarm.py`.

Move these from `src/isaac_openarm_teleop.py`:

- OpenArm joint name lists
- gripper joint name lists
- preferred IK seed configs
- robot prim discovery paths
- Lula IK solver initialization
- joint index mapping
- left/right IK calls
- gripper target mapping
- camera prim paths

OpenArm config file: `config/robots/openarm.yaml`.

Suggested config:

```yaml
robot_type: openarm
usd: "robot_configs/openarm_config/openarm_bimanual/openarm_bimanual_env.usd"
urdf: "robot_configs/openarm_config/openarm_bimanual_stl.urdf"
left_arm_config: "robot_configs/openarm_config/left_arm"
right_arm_config: "robot_configs/openarm_config/right_arm"

prim_search_paths:
  - "/World/Robot"
  - "/World/openarm"
  - "/openarm"
  - "/Robot"
  - "/Environment/openarm"
  - "/World/openarm_bimanual"

left_arm:
  frame_name: "openarm_left_hand"
  joints:
    - "openarm_left_joint1"
    - "openarm_left_joint2"
    - "openarm_left_joint3"
    - "openarm_left_joint4"
    - "openarm_left_joint5"
    - "openarm_left_joint6"
    - "openarm_left_joint7"
  preferred_config: [0.0, -1.0, 0.0, 1.2, 0.0, 0.0, 0.0]
  workspace_offset: [0.0, 0.15, 0.0]

right_arm:
  frame_name: "openarm_right_hand"
  joints:
    - "openarm_right_joint1"
    - "openarm_right_joint2"
    - "openarm_right_joint3"
    - "openarm_right_joint4"
    - "openarm_right_joint5"
    - "openarm_right_joint6"
    - "openarm_right_joint7"
  preferred_config: [0.0, 1.0, 0.0, 1.2, 0.0, 0.0, 0.0]
  workspace_offset: [0.0, -0.15, 0.0]

grippers:
  open_position: 0.132
  closed_position: -1.0
  speed: 0.05
  threshold: 0.5
  left_joints:
    - "openarm_left_finger_joint1"
    - "openarm_left_finger_joint2"
  right_joints:
    - "openarm_right_finger_joint1"
    - "openarm_right_finger_joint2"

cameras:
  head:
    prim_path: "/openarm/openarm_body_link/head_camera"
    topic: "/camera/head/image_raw"
  wrist_left:
    prim_path: "/openarm/openarm_left_link7/left_wrist_camera"
    topic: "/camera/wrist_left/image_raw"
  wrist_right:
    prim_path: "/openarm/openarm_right_link7/right_wrist_camera"
    topic: "/camera/wrist_right/image_raw"
```

Acceptance criteria:

- OpenArm still launches and behaves as before.
- Hardcoded OpenArm joint/camera/frame lists are removed from launch script.
- OpenArm adapter can be instantiated from YAML.

### 3.3 Panda Adapter

Create `src/robot_adapters/panda.py`.

Move Panda-specific code from `src/isaac_panda_teleop.py`:

- Franka loading
- supported motion policy config loading
- IK frame name `panda_hand`
- gripper positions
- workspace bounds
- camera path

Panda config file: `config/robots/panda.yaml`.

Acceptance criteria:

- Panda workflow still works.
- Panda script becomes a thin launcher using the same teleop core.

## Phase 4: Isaac Backend Extraction

Goal: isolate Isaac Sim imports and lifecycle code.

### 4.1 Isaac App Manager

Create `src/isaac_backend/app.py`.

Responsibilities:

- create `SimulationApp`
- enable `omni.isaac.ros2_bridge`
- load stage
- create `World`
- warm up/stabilize simulation
- hide UI panels, if configured
- close cleanly

Suggested API:

```python
class IsaacApp:
    def __init__(self, app_config):
        ...

    def start(self):
        ...

    def load_stage(self, usd_path: str):
        ...

    def create_world(self):
        ...

    def step(self, render: bool = True):
        ...

    def is_running(self) -> bool:
        ...

    def close(self):
        ...
```

Important: Isaac imports must happen after `SimulationApp` is created, following Isaac Sim requirements.

Acceptance criteria:

- Launch scripts no longer duplicate warmup/stage/world setup.
- Isaac-specific imports are not pulled into pure unit tests.

### 4.2 ROS Publishers

Create `src/isaac_backend/ros_publishers.py`.

Responsibilities:

- publish `JointState`
- publish camera `Image`
- optionally publish diagnostic metrics later

Acceptance criteria:

- Joint state publishing moves out of main OpenArm loop.
- Message construction is reusable.

### 4.3 Camera Manager

Create `src/isaac_backend/camera_manager.py`.

Responsibilities:

- discover view cameras for viewport switching
- configure recording cameras from robot adapter camera specs
- create replicator render products
- publish camera frames asynchronously
- track dropped camera frames and exceptions

Required behavior:

- No silent exception swallowing without at least throttled diagnostics.
- Camera FPS, resolution, and queue size must be configurable.
- Camera manager can be disabled entirely for performance tuning.

Suggested config:

```yaml
cameras:
  enabled: true
  resolution: [480, 360]
  publish_interval_frames: 2
  queue_size: 3
  log_errors: true
```

Acceptance criteria:

- Existing `/camera/*/image_raw` topics still publish for OpenArm.
- Camera publishing can be disabled with config.
- Main control loop remains readable.

## Phase 5: Teleop Session Orchestration

Goal: replace large procedural loops with a session object.

Create `src/teleop_core/session.py`.

Responsibilities:

- consume current left/right controller states
- update calibration
- compute robot-space target poses
- apply filtering
- apply safety
- produce `TeleopTargets`

Suggested dataclasses:

```python
@dataclass
class EndEffectorTarget:
    position_xyz: np.ndarray
    orientation_wxyz: np.ndarray
    valid: bool

@dataclass
class GripperTarget:
    closed: bool
    analog_value: float

@dataclass
class BimanualTeleopTargets:
    left_ee: EndEffectorTarget
    right_ee: EndEffectorTarget
    left_gripper: GripperTarget
    right_gripper: GripperTarget
```

OpenArm launch loop after refactor should conceptually look like:

```python
while app.is_running():
    controller_provider.spin_once()
    controller_states = controller_provider.latest()

    targets = teleop_session.update(controller_states)
    if targets.ready:
        action = robot_adapter.compute_action(targets)
        robot_adapter.apply_action(action)
        joint_state_pub.publish(robot_adapter.get_joint_names(), action.positions)

    camera_manager.update()
    app.step(render=True)
```

Acceptance criteria:

- Main launcher is under roughly 150-200 lines.
- Robot-specific and teleop-specific code is not mixed in the launcher.

## Phase 6: Remote Server and VPN Performance

Goal: make the system robust when Isaac Sim runs remotely over VPN.

### 6.1 Edge Ingress Option

The most robust deployment is:

```text
Quest 3
  -> local workstation/laptop on same LAN
  -> compact state stream over VPN
  -> remote Isaac server
```

Do not require the Quest to connect directly to the remote VPN server if the headset cannot route cleanly over VPN.

Add a mode where `webxr_ros_bridge.py` can run locally as an ingress node and forward state to the remote server.

Potential options:

1. Keep ROS 2 over VPN if DDS discovery and QoS are reliable.
2. Add explicit WebSocket forwarding from local ingress to remote teleop server.
3. Use Zenoh later if ROS 2 DDS over VPN becomes unreliable.

Recommended first step:

- Keep the current WSS receiver near the Quest.
- Forward compact JSON or MessagePack state to a remote receiver on the Isaac server.

### 6.2 Latency Metrics

Track:

- browser send timestamp
- ingress receive timestamp
- remote receive timestamp
- control-loop consume timestamp
- Isaac apply timestamp

Expose logs:

```text
[Transport] rx=89.8Hz drop=0.2% jitter=4.1ms age_p50=31ms age_p95=74ms
[Control] ik=58.9Hz left_success=98.7% right_success=97.9% stale=0
```

Acceptance criteria:

- Operator can see if poor behavior comes from packet loss, VPN jitter, IK failure, or Isaac frame time.

### 6.3 Prediction and Jitter Buffer

After metadata exists, add optional prediction:

- Estimate controller linear velocity from recent poses.
- If packet age is small but nonzero, predict position forward by age.
- Cap prediction horizon, for example 50-80 ms.

Add optional jitter buffer:

- For highly jittery VPN, buffer 1-3 frames and consume states at a steady rate.
- Keep disabled by default if it adds too much perceived latency.

Acceptance criteria:

- Prediction can be enabled/disabled from config.
- Prediction never extrapolates beyond configured horizon.

## Phase 7: Replay and Testing Workflow

Goal: make tuning possible without always using the Quest.

### 7.1 Controller Stream Recorder

Add script:

```text
scripts/record_quest_stream.sh
```

or Python module:

```text
src/tools/record_controller_stream.py
```

Record incoming controller states as JSONL:

```json
{"t": 0.000, "left": {...}, "right": {...}}
{"t": 0.011, "left": {...}, "right": {...}}
```

Store under:

```text
recordings/controller_streams/
```

Add `.gitignore` entry for recordings.

### 7.2 Replay Tool

Add:

```text
src/tools/replay_controller_stream.py
```

Modes:

- replay to ROS topics
- replay directly into teleop core tests
- replay at original speed or fixed speed

Acceptance criteria:

- A recorded Quest session can drive OpenArm in Isaac without wearing the headset.
- Filter and retargeting changes can be compared against the same input stream.

### 7.3 Unit Tests

Add focused tests for pure modules:

- calibration sample averaging
- frame transform position mapping
- quaternion convention conversion
- SLERP sign handling
- velocity limiting
- stale packet detection
- robot config loading

Suggested test layout:

```text
tests/
  test_calibration.py
  test_frame_transforms.py
  test_filters.py
  test_safety.py
  test_robot_config.py
```

Acceptance criteria:

- Tests run without Isaac Sim.
- Isaac-dependent code is skipped or integration-tested separately.

## Phase 8: Configuration System

Goal: make robot/session behavior configurable and validated.

Create `src/config_loader.py` or `src/teleop_core/config.py`.

Responsibilities:

- load `config/config.yaml`
- load selected robot config from `config/robots/*.yaml`
- resolve relative paths against repo root
- validate required keys
- provide defaults

Suggested main config:

```yaml
active_robot: openarm

paths:
  isaac_sim: "/home/saurabh/isaac_sim"
  certs:
    cert: "certs/cert.pem"
    key: "certs/key.pem"

server:
  host: "0.0.0.0"
  websocket_port: 9999
  https_port: 8000

isaac:
  headless: false
  width: 1920
  height: 1080
  window_width: 1920
  window_height: 1080
  hide_ui_panels: true

teleop:
  calibration_samples: 30
  position_scale: [1.0, 1.0, 1.0]
  workspace_center: [0.3, 0.0, 0.3]
  smoothing:
    position_alpha: 0.9
    orientation_alpha: 0.9
  deadman_timeout_ms: 250
  hard_timeout_ms: 1000
  max_target_jump_m: 0.25

transport:
  log_interval_s: 3.0
  enable_prediction: false
  prediction_horizon_ms: 50

cameras:
  enabled: true
  resolution: [480, 360]
  publish_interval_frames: 2
  queue_size: 3
```

Acceptance criteria:

- Scripts and launch modules use the same config loader.
- Relative paths resolve correctly from repo root.
- Missing required robot config produces a clear error.

## Phase 9: Launch Scripts and Backward Compatibility

Goal: preserve familiar commands.

Keep:

```bash
./scripts/run_wireless.sh
./scripts/run_openarm_teleop.sh
./scripts/run_panda_teleop.sh
```

But make them call:

```bash
python -m src.launch.webxr_bridge
$ISAAC_SIM_PATH/python.sh -m src.launch.openarm_teleop
$ISAAC_SIM_PATH/python.sh -m src.launch.panda_teleop
```

If Python module execution is awkward under Isaac Sim, call the file path directly:

```bash
$ISAAC_SIM_PATH/python.sh src/launch/openarm_teleop.py
```

Acceptance criteria:

- Existing user commands still work.
- New launch modules support CLI arguments:
  - `--config config/config.yaml`
  - `--robot openarm`
  - `--headless`
  - `--disable-cameras`
  - `--debug-ik`

## Implementation Order

Recommended order:

1. Phase 1.1: unify config, ports, docs, and launch scripts.
2. Phase 1.2: add sequence/timestamp metadata and bridge metrics.
3. Phase 1.3: add deadman timeout.
4. Phase 1.4: replace quaternion smoothing with SLERP.
5. Phase 2.1-2.4: extract pure teleop modules.
6. Phase 3.1-3.2: create base adapter and OpenArm adapter.
7. Phase 4: extract Isaac app and camera manager.
8. Phase 5: add teleop session orchestrator and simplify OpenArm launcher.
9. Phase 3.3: migrate Panda onto the same path.
10. Phase 7: add record/replay and tests.
11. Phase 6: optimize remote/VPN deployment.

This order keeps the working OpenArm workflow intact while steadily reducing risk.

## Definition of Production-Like Behavior

The refactor is considered successful when:

- OpenArm teleoperation still works with the Quest 3 inside Isaac Sim 5.1.0.
- The main OpenArm launcher is mostly orchestration, not business logic.
- OpenArm-specific robot details live in `config/robots/openarm.yaml` and `robot_adapters/openarm.py`.
- Transport metrics show packet rate, drops, jitter, and stale data.
- Control metrics show IK success/failure rates and loop rate.
- A dropped WebSocket/VPN connection does not produce robot jumps.
- Camera publishing can be disabled or rate-limited without touching control logic.
- A recorded controller stream can be replayed into the system.
- Adding another robot does not require copying `isaac_openarm_teleop.py`; it requires a config file and a focused adapter.

## Notes for GPT-5.4 Implementation

When making code changes:

- Preserve existing topic names and command-line workflows first.
- Avoid importing Isaac Sim modules in pure modules or unit tests.
- Keep any Isaac imports after `SimulationApp` is created.
- Prefer small, verifiable commits or patches by phase.
- Do not remove the existing scripts until replacements are proven.
- Avoid broad style-only refactors while behavior is being migrated.
- Add tests around pure functions before changing control behavior.
- Keep all robot embodiment constants out of generic teleop modules.
- Log errors with throttling instead of silently swallowing exceptions.
- If behavior changes intentionally, document the operator-visible effect in README/docs.

