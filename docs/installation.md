# Installation Guide

Complete guide to set up TeleSim — the VR teleoperation and dataset collection platform for Isaac Sim 6.0.

---

## Prerequisites

### Hardware

| Component | Requirement |
|-----------|-------------|
| **Operating System** | Ubuntu 24.04 |
| **GPU** | NVIDIA RTX (RTX 3080 or better recommended) |
| **VR Headset** | Any WebXR-capable browser on a VR headset (tested on Meta Quest 3). Must be on the same WiFi network as the PC, or connected via VPN. |

### Software

| Software | Version | Notes |
|----------|---------|-------|
| **NVIDIA Isaac Sim** | 6.0 | Install separately — [NVIDIA Omniverse](https://developer.nvidia.com/isaac/sim) |
| **ROS 2 Jazzy** | — | Bundled with Isaac Sim 6.0 — **no separate install needed** |
| **Python** | 3.10+ | Used for the virtual environment and recording worker |
| **uv** | Latest | Fast Python package manager — [install uv](https://docs.astral.sh/uv/getting-started/installation/) |

> **No extra robot assets needed**: All USD/URDF files for OpenArm, AC One, and FFW BG2 are bundled
> under `robot_configs/` in the repository. You only need Isaac Sim itself.

---

## Step-by-Step Installation

### Step 1 — Clone the repository

```bash
git clone https://github.com/AiSaurabhPatil/telesim.git
cd telesim
```

### Step 2 — Create the virtual environment with `uv`

TeleSim uses a single `.venv` for both the main pipeline and the LeRobot dataset recording worker.
Isaac Sim's bundled `python.sh` spawns the recording worker as a subprocess using this venv's Python.

```bash
# Create the venv (--system-site-packages gives access to system libs if needed)
uv venv .venv --system-site-packages

# Activate it
source .venv/bin/activate

# Install all dependencies
uv pip install -r requirements.txt
```

> **Why `uv`?** `uv` resolves and installs packages significantly faster than `pip` and generates
> a reproducible lock for the environment. All subsequent commands assume the venv is activated.

### Step 3 — Generate SSL certificates (one-time)

WebXR requires HTTPS. Generate a self-signed certificate that includes your workstation's LAN IP
as a **Subject Alternative Name (SAN)** — this prevents the `ERR_CERT_COMMON_NAME_INVALID` error
on the Quest browser.

```bash
bash scripts/generate_cert.sh   # auto-detects your LAN IP
```

This creates `certs/cert.pem` and `certs/key.pem`.

> **Re-run whenever your workstation's IP changes** (e.g., after reconnecting to WiFi):
> ```bash
> rm -f certs/cert.pem certs/key.pem && bash scripts/generate_cert.sh
> ```

### Step 4 — Configure `config/config.yaml`

Open `config/config.yaml` and set the path to your Isaac Sim 6.0 installation and the active robot:

```yaml
# Which robot to load by default
active_robot: acone   # openarm | acone | ffw_bg2

paths:
  # Absolute path to your Isaac Sim 6.0 installation directory
  isaac_sim: "/home/<your-username>/isaac_sim"

  # SSL certificates (generated in Step 3)
  certs:
    cert: "certs/cert.pem"
    key:  "certs/key.pem"
```

> **Only `paths.isaac_sim` must be changed.** All other defaults are sensible for a first run.

---

## Configuration Reference

### `config/config.yaml` — Global Settings

This file controls server networking, Isaac Sim launch options, camera capture, the recording pipeline, and latency-tuning knobs. Key sections:

```yaml
active_robot: acone    # openarm | acone | ffw_bg2

paths:
  isaac_sim: "/home/<you>/isaac_sim"   # ← Must set this
  certs:
    cert: "certs/cert.pem"
    key:  "certs/key.pem"

server:
  host: "0.0.0.0"
  websocket_port: 9999   # WebSocket for controller data
  https_port: 8000       # HTTPS for WebXR page

lan:
  robot_pc_ip: ""        # auto-detected if empty
  websocket_port: 9999
  https_port: 8000

isaac:
  headless: false
  target_control_rate_hz: 60        # Control loop target rate
  render_every_n_steps: 1           # Set >1 to skip renders and speed up the loop
  width: 1920
  height: 1080

cameras:
  enabled: true
  resolution: [640, 480]            # Capture resolution for camera images
  publish_interval_frames: 2        # Publish every Nth frame to ROS

recording:
  enabled: false                    # Set true or pass --record to enable
  dataset_format: "v3.0"           # Do not change — only v3.0 is supported
  root: "datasets"
  repo_id: "local/quest3-openarm"
  task: "Teleoperate OpenArm to complete the task"
  fps: 30
  max_episodes: null
  deferred_rendering: true          # Skip camera capture during teleop
  cameras:
    resolution: [640, 480]
  buttons:
    switch_camera: "right_primary"
    reset_scene:   "right_secondary"
    save_episode:  "left_primary"
    start_episode: "left_secondary"

transport:
  enable_prediction: true
  prediction_horizon_ms: 40         # Tune to your measured one-way latency
```

### `config/robots/<robot>.yaml` — Per-Robot Settings

Each robot has its own YAML file that overrides or extends the global config for that specific
robot's kinematics, cameras, and safety limits. The following sections are defined per robot:

| Section | Description |
|---------|-------------|
| `usd` / `urdf` | Paths to the scene USD and URDF files (relative to project root) |
| `left_arm` / `right_arm` | Joint names and preferred IK seed configuration |
| `grippers` | Open/closed positions, speed, and joint names |
| `cameras` | USD prim paths and ROS topic names for each camera |
| `ik` | IK solver options (orientation mode, tolerances, fallback) |
| `teleop` | Calibration, smoothing time constants, safety limits |
| `recording` | Joint groups mapped to dataset state/action vectors |
| `domain_randomization` | Object placement and lighting ranges (FFW BG2 enabled by default) |

Example snippet from `config/robots/acone.yaml`:

```yaml
robot_type: acone
display_name: "AC One"
usd:  "robot_configs/acone_config/Collected_acone_scene_v3/acone_scene_v3.usd"
urdf: "robot_configs/acone_config/acone_with_tcp.urdf"

left_arm:
  frame_name: "left_tcp"
  joints: [left_joint1, left_joint2, left_joint3, left_joint4, left_joint5, left_joint6]
  preferred_config: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

grippers:
  open_position: 0.04
  closed_position: 0.0
  speed: 0.02
  threshold: 0.5
  left_joints:  [left_joint7, left_joint8]
  right_joints: [right_joint17, right_joint18]

cameras:
  head_camera:
    prim_path: "/World/ACone/Geometry/base_link/head_camera"
    topic: "/camera/head_camera/image_raw"
  left_wrist_camera:
    prim_path: "/World/ACone/Geometry/base_link/.../left_wrist_camera"
    topic: "/camera/left_wrist_camera/image_raw"

ik:
  orientation_mode: "full_pose"
  position_tolerance: 0.003
  orientation_tolerance: 0.25
  orientation_fallback_to_position: true

teleop:
  calibration_samples: 30
  smoothing:
    position_tau_s: 0.10
    orientation_tau_s: 0.10
  deadman_timeout_ms: 500
  hard_timeout_ms: 1000
  max_target_jump_m: 0.25
  max_target_velocity_mps: 0.4
```

---

## Verify Installation

### Test 1: Config loader

```bash
source .venv/bin/activate
python src/config_loader.py get --config config/config.yaml paths.isaac_sim
```

Should print the Isaac Sim path you configured.

### Test 2: Wireless bridge only (no Isaac Sim needed)

```bash
./scripts/run_wireless.sh
```

Should print the HTTPS server URL and WebSocket URL with your workstation's LAN IP.

### Test 3: Full launch (requires Isaac Sim)

```bash
./scripts/run_lan_teleop.sh --robot acone
```

Isaac Sim should open and print `[Init] Warming up Isaac Sim...`.

---

## Next Steps

Once installed, proceed to the [Usage Guide](usage.md) to start teleoperating robots and recording datasets.
