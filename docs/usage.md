# Usage Guide

Complete guide to running VR teleoperation, recording datasets, and rendering camera observations with TeleSim.

---

## Quick Start — LAN Teleop

The simplest path when the Quest headset and workstation are on the **same WiFi network**:

```bash
./scripts/run_lan_teleop.sh --robot acone
```

This single command starts three processes:

1. **HTTPS server** (port 8000) — serves the WebXR page to the Quest browser
2. **WebSocket bridge** (port 9999) — receives controller data and publishes ROS 2 topics
3. **Robot teleop** — opens Isaac Sim with the windowed GUI on this workstation's monitor

**On your Quest browser**, navigate to:
```
https://<WORKSTATION_LAN_IP>:8000/web/webxr_streamer.html
```

Accept the self-signed certificate warning (click **Advanced → Proceed**), then tap **"Start AR Session"**.

> **Windowed GUI is the low-latency default.** Viewing the sim directly on the workstation monitor
> has lower visual latency than encoding to WebRTC and decoding in a client. Control-loop speedup
> comes from render throttling (`isaac.render_every_n_steps` in `config.yaml`), which renders only
> every Nth physics step regardless of display mode.

---

## Calibration

1. Isaac Sim loads and prints `[Init] Warming up Isaac Sim...`
2. Start the AR session on the Quest
3. **Hold both controllers steady** in a comfortable neutral pose for ~1 second (30 samples by default)
4. The system prints `CALIBRATION COMPLETE` for each arm
5. Your current hand position becomes the robot's workspace origin — move your hands to control the robot

> **Calibration tip**: Sit or stand in the pose you intend to operate in. All movements are relative
> to the calibrated origin, so a comfortable starting pose gives you the most workspace range.
> To recalibrate, restart the script.

---

## Controller Buttons

| Button | Action |
|--------|--------|
| **Left Secondary** (Y / X) | **Start** a new recording episode |
| **Left Primary** (B / A) | **Save** the current episode |
| **Right Secondary** | **Reset scene** (discards unsaved episode when deferred rendering is on) |
| **Right Primary** | **Cycle camera views** (perspective → head → left wrist → right wrist) |
| **Left / Right Trigger** | Close the respective gripper |
| **Left / Right Grip Hold** | Engage clutched arm control when clutch mode is enabled |
| **Left / Right Controller Movement** | Move the respective arm's end-effector |

---

## Supported Robots

All three robots are launched with the same `run_lan_teleop.sh` script using `--robot <name>`:

```bash
./scripts/run_lan_teleop.sh --robot openarm
./scripts/run_lan_teleop.sh --robot acone
./scripts/run_lan_teleop.sh --robot ffw_bg2
```

Or use the individual per-robot scripts directly:

```bash
bash scripts/run_openarm_teleop.sh
bash scripts/run_acone_teleop.sh
bash scripts/run_ffw_bg2_teleop.sh
```

| Robot | Arms | DOF | Domain Randomization |
|-------|------|-----|----------------------|
| **OpenArm Bimanual** | L+R 7-DOF | 14 arm + 2 gripper (scalar) | Off by default |
| **AC One** | L+R 6-DOF | 12 arm + 4 gripper | Off by default |
| **FFW BG2** | L+R 7-DOF | 14 arm + 8 gripper | **On by default** |

---

## Recording Datasets

Dataset recording is a **two-phase process** that decouples the live control loop from camera rendering.

### Phase 1 — Teleop + State Recording

Enable recording by passing `--record` (or setting `recording.enabled: true` in `config.yaml`):

```bash
./scripts/run_lan_teleop.sh \
  --robot acone \
  --record \
  --dataset-repo-id local/quest3-acone \
  --task "sort nuts and bolts in different bins" \
  --recording-fps 30 \
  --max-episodes 10
```

**What gets saved during teleop:**

- Joint positions and actions for every frame (Parquet format, written by `worker_process.py`)
- A per-episode sidecar JSON with domain randomization state (`meta/episodes/scene_ep_NNNNNN.json`)
- **Camera images are NOT captured** — this keeps the GPU free for physics and IK

The dataset is written to `datasets/local/quest3-acone/` by the `worker_process.py` subprocess
running in `.venv`. Isaac Sim's Python is never polluted with LeRobot's dependencies.

**Episode workflow during teleop:**

1. Press **Left Secondary** (Y/X) to **start** a new episode
2. Perform the task with both arms
3. Press **Left Primary** (B/A) to **save** the episode
4. Press **Right Secondary** to **reset** the scene before starting the next episode

### Phase 2 — Deferred Camera Rendering

After your teleop session, run the renderer to replay joint trajectories and capture camera frames:

```bash
./scripts/run_deferred_renderer.sh \
  local/quest3-acone \
  local/quest3-acone-rendered \
  --robot-type acone
```

**What the renderer does:**

1. Reads joint trajectories from the source dataset via `extract_states.py`
2. Starts Isaac Sim in **headless mode** (no display needed — can run on a headless server)
3. For each episode:
   - Resets the world to USD default poses
   - Restores the exact domain randomization state from the sidecar JSON
   - Replays joints with correct physics substep timing to match the original control rate
   - Captures RTX camera frames and writes them to the output dataset
4. Writes the final LeRobot v3.0 dataset to `datasets/local/quest3-acone-rendered/`

> **Physics substep auto-detection**: The renderer calculates how many physics steps to run per
> frame based on `isaac.target_control_rate_hz` and `recording.fps` from your config.
> Override with `--physics-substeps N` if needed.

### Dataset Format

| Property | Value |
|----------|-------|
| **Format** | LeRobot **v3.0** only (`dataset_format: v3.0` in config) |
| **Location** | `datasets/<repo-id>/` |
| **Camera features** | `observation.images.<camera_name>` (video encoded) |
| **State / action** | `observation.state`, `action` (float32 vectors) |
| **Scene sidecar** | `meta/episodes/scene_ep_NNNNNN.json` (domain randomization replay data) |

---

## Network Modes

### Mode 1 — LAN Direct (Recommended)

Quest and workstation on the **same WiFi network**. Isaac Sim renders on the local monitor.

```bash
./scripts/run_lan_teleop.sh --robot acone
```

### Mode 1b — LAN + WebRTC Viewport Streaming

Stream the Isaac Sim viewport to a **different device** over the network using WebRTC:

```bash
./scripts/run_lan_teleop.sh --robot acone --webrtc
```

Connect with the [Isaac Sim Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_streaming_client.html)
on another device pointing at `<PC_IP>:49100`.

### Mode 2 — VPN / Remote (Isaac Sim on a separate remote PC)

Run a local ingress near the Quest, and forward controller packets over VPN to a remote machine
running Isaac Sim.

**Local PC — Terminal 1 (HTTPS server)**

```bash
source .venv/bin/activate
python3 web/https_server.py 8000 --cert certs/cert.pem --key certs/key.pem
```

**Local PC — Terminal 2 (ingress bridge)**

```bash
source .venv/bin/activate
python3 -m src.launch.webxr_bridge \
  --mode ingress \
  --host 0.0.0.0 \
  --port 9999 \
  --cert certs/cert.pem \
  --key certs/key.pem \
  --forward-url wss://<REMOTE_PC_IP>:9998 \
  --forward-insecure
```

**Remote PC — Terminal 1 (remote receiver)**

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
python3 -m src.launch.webxr_bridge \
  --mode remote-receiver \
  --host 0.0.0.0 \
  --port 9998
```

**Remote PC — Terminal 2 (robot teleop)**

```bash
bash scripts/run_acone_teleop.sh
```

The transport bridge logs packet rate, drop percentage, jitter, and end-to-end packet age to
help diagnose remote-control issues.

### Mode 3 — Wireless Only (Bridge + HTTPS, no Isaac Sim)

WebXR → ROS bridge without starting Isaac Sim. Useful for connecting to an external ROS stack:

```bash
./scripts/run_wireless.sh
```

### Mode 4 — Manual (3 separate terminals)

For advanced debugging or custom setups:

**Terminal 1 — HTTPS server**
```bash
source .venv/bin/activate
python3 web/https_server.py 8000 --cert certs/cert.pem --key certs/key.pem
```

**Terminal 2 — WebSocket bridge (direct mode)**
```bash
source .venv/bin/activate
# Use Isaac Sim's bundled rclpy (no separate ROS install needed)
export PYTHONPATH="/path/to/isaac_sim/exts/isaacsim.ros2.core/jazzy/rclpy:$PYTHONPATH"
export LD_LIBRARY_PATH="/path/to/isaac_sim/exts/isaacsim.ros2.core/jazzy/lib:$LD_LIBRARY_PATH"
python3 -m src.launch.webxr_bridge \
  --mode direct \
  --host 0.0.0.0 \
  --port 9999 \
  --cert certs/cert.pem \
  --key certs/key.pem
```

**Terminal 3 — Robot teleop**
```bash
bash scripts/run_acone_teleop.sh
```

---

## Latency Tuning

The system logs transport and control metrics every ~3 seconds:

```
[Transport][direct] rx=89.8Hz drop=0.2% jitter=4.1ms age_p50=31ms age_p95=74ms
[Control] loop=110.0Hz left_success=99.0% left_ik_ms=3.2 left_browser_age_ms=45 left_age_ms=12
```

**Metrics explained:**

| Metric | Description | Target |
|--------|-------------|--------|
| `loop Hz` | Control loop rate | ≥ `target_control_rate_hz` |
| `age_p50` | Quest→bridge median latency | < 50 ms on LAN |
| `age_p95` | Quest→bridge 95th-percentile latency | < 100 ms |
| `left_browser_age_ms` | End-to-end Quest→applied latency | < 50 ms |
| `drop%` | Packet drop rate | < 1% |
| `left_ik_ms` | IK solve time per frame | < 5 ms |

**Key tuning knobs in `config/config.yaml`:**

| Config Key | Effect |
|-----------|--------|
| `isaac.target_control_rate_hz` | Target control loop rate (higher = more responsive) |
| `isaac.render_every_n_steps` | Render only every Nth step (higher = faster loop, less smooth viewport) |
| `transport.enable_prediction` | Enable pose prediction to cancel one-way transport lag |
| `transport.prediction_horizon_ms` | Set to your measured `age_p50` value |
| `teleop.smoothing.position_tau_s` | Smoothing time constant (~0.10 s = stable, lower = snappier) |
| `teleop.stale_recovery_alpha` | Blend speed after a controller data hiccup |

**Analyze a captured log:**

```bash
python scripts/summarize_metrics.py /path/to/logfile.txt
```

---

## Domain Randomization

Domain randomization randomizes object positions and scene lighting at the start of each episode.
The exact randomization is saved in a sidecar JSON and replayed faithfully during deferred rendering
so camera images match what the operator saw.

**FFW BG2** has domain randomization enabled by default in `config/robots/ffw_bg2.yaml`.
OpenArm and AC One have it commented out — uncomment to enable.

Enable it in `config/robots/<robot>.yaml`:

```yaml
domain_randomization:
  enabled: true
  seed: null        # null = random seed per episode
  settle_steps: 30  # physics steps to wait for objects to settle

  objects:
    bounds:
      reference_prim: "/World/TablePrim"   # USD prim used as placement surface
      center_area_scale: [0.8, 0.8]
    min_distance_m: 0.18
    items:
      - path: "/World/Bowl"
        radius_m: 0.11
        yaw_deg: [-30.0, 30.0]
        z_offset_m: 0.0
      - path: "/World/Apple"
        radius_m: 0.05
        yaw_deg: [-180.0, 180.0]
        z_offset_m: 0.0

  lighting:
    lights:
      - path: "/World/Environment/DistantLight"
        intensity: [1000.0, 5000.0]
        exposure_jitter: [-0.5, 0.5]
        color_temperature: [4500.0, 7000.0]
```

---

## Visualizing Datasets

Use the bundled [Rerun](https://rerun.io/)-based visualizer to inspect joint trajectories and
camera frames side-by-side:

```bash
./scripts/visualize_lerobot_dataset.sh \
  --repo-id local/quest3-acone \
  --root datasets/local/quest3-acone \
  --episode-index 0
```

---

## ROS 2 Topics Reference

### Controller Topics (published by WebSocket bridge from Quest)

| Topic | Type | Description |
|-------|------|-------------|
| `/quest/left_hand/pose` | `geometry_msgs/PoseStamped` | Left controller 6-DoF pose |
| `/quest/right_hand/pose` | `geometry_msgs/PoseStamped` | Right controller 6-DoF pose |
| `/quest/left_hand/inputs` | `sensor_msgs/Joy` | Left controller buttons and axes |
| `/quest/right_hand/inputs` | `sensor_msgs/Joy` | Right controller buttons and axes |

### Joy Message Mapping

```python
# Axes (float values)
axes[0] = trigger       # 0.0 to 1.0
axes[1] = squeeze/grip  # 0.0 to 1.0
axes[2] = thumbstick_x  # -1.0 to 1.0
axes[3] = thumbstick_y  # -1.0 to 1.0

# Buttons (0 or 1)
buttons[0] = A / X button
buttons[1] = B / Y button
buttons[2] = Menu button
buttons[3] = Thumbstick click
```

### Robot Topics (published by Isaac Sim)

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | All robot joint positions |
| `/camera/head_camera/image_raw` | `sensor_msgs/Image` | Head camera |
| `/camera/left_wrist_camera/image_raw` | `sensor_msgs/Image` | Left wrist camera |
| `/camera/right_wrist_camera/image_raw` | `sensor_msgs/Image` | Right wrist camera |
| `/camera/perspective_camera/image_raw` | `sensor_msgs/Image` | Viewport perspective camera |

Verify topics are publishing:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep quest
ros2 topic hz /quest/right_hand/pose   # Should show ~60-90 Hz
ros2 topic echo /quest/right_hand/pose
```

---

## Next Steps

- See [Troubleshooting](troubleshooting.md) if you encounter issues.
- To add a new robot, create a `config/robots/<name>.yaml`, a `src/robot_adapters/<name>.py`
  implementing `RobotAdapter`, and a `scripts/run_<name>_teleop.sh`.
