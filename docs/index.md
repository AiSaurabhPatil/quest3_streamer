# TeleSim

**TeleSim** is a VR teleoperation and dataset collection platform for **NVIDIA Isaac Sim 6.0**.
Control multiple bimanual robots using any **WebXR-compatible VR headset** (tested on Meta Quest 3)
and record datasets in **LeRobot v3.0 format** — with a fully decoupled, latency-optimized recording pipeline.

---

## What TeleSim Does

TeleSim streams 6-DoF controller poses from a **WebXR-compatible VR headset** over WebSocket to
ROS 2 topics, which Isaac Sim consumes to run real-time inverse-kinematics on bimanual robot arms.
A key architectural feature is **deferred rendering**: camera images are *not* captured during the
live teleoperation loop — keeping the control loop fast — and are rendered offline in a separate
Isaac Sim headless pass after the session ends.

### Key Capabilities

| Feature | Description |
|---------|-------------|
| 🤖 **Multi-robot support** | OpenArm Bimanual, AC One, FFW BG2 — all from the same codebase |
| ⚡ **Decoupled recording** | Joint trajectories saved live; camera frames rendered offline to avoid GPU latency |
| 📦 **LeRobot v3.0 datasets** | Out-of-the-box dataset writing with a subprocess worker, compatible with HuggingFace |
| 🎲 **Domain randomization** | Per-episode object and lighting randomization with exact scene replay during deferred rendering |
| 🌐 **Flexible networking** | LAN direct, VPN/remote ingress, and WebRTC viewport streaming |
| 📊 **Built-in latency metrics** | Real-time control loop and transport diagnostics |

---

## System Architecture

The platform is built around two loosely coupled stages that run sequentially:

```
Stage 1 — Live Teleoperation
──────────────────────────────────────────────────────────────────────────────
  WebXR Headset
       │  HTTPS + WebSocket (port 8000 / 9999)
       ▼
  WebSocket Bridge  ──► ROS 2 Topics (PoseStamped, Joy)
       │
       ▼
  Isaac Sim 6.0  ──► Bimanual IK loop  ──► Joint positions
                                                  │
                                                  ▼
                                       LeRobot Recorder (IPC pipe)
                                                  │
                                                  ▼
                                       Worker subprocess (.venv)
                                                  │
                                                  ▼
                                      Raw dataset (Parquet + sidecar JSON)
                                      [Camera images NOT yet captured]

Stage 2 — Deferred Camera Rendering  (offline, after session ends)
──────────────────────────────────────────────────────────────────────────────
  Raw dataset  ──► extract_states.py  ──► per-episode joint trajectories
       │
       ▼
  Isaac Sim (headless)  ──► replay joints  ──► RTX Camera Manager
                                                      │
                                                      ▼
                                      Final LeRobot v3.0 Dataset
                                      (Parquet + rendered camera videos)
```

> **Why deferred rendering?** GPU rendering is the single biggest source of latency in the
> control loop. By skipping camera capture during teleop and replaying joint trajectories offline,
> the live loop runs at 60–120 Hz while still producing high-quality camera observations in the dataset.

---

## Supported Robots

All robots run under **ROS 2 Jazzy** (bundled with Isaac Sim 6.0).

| Robot | Launch Script | Config File | Arms | Total DOF |
|-------|--------------|-------------|------|-----------|
| **OpenArm Bimanual** | `run_openarm_teleop.sh` | `robots/openarm.yaml` | L+R 7-DOF | 14 arm + 2 gripper (scalar) |
| **AC One** | `run_acone_teleop.sh` | `robots/acone.yaml` | L+R 6-DOF | 12 arm + 4 gripper |
| **FFW BG2** | `run_ffw_bg2_teleop.sh` | `robots/ffw_bg2.yaml` | L+R 7-DOF | 14 arm + 8 gripper |

Each robot has head, left-wrist, and right-wrist cameras plus a perspective viewport camera.

> **No extra robot assets needed**: All USD/URDF files for OpenArm, AC One, and FFW BG2 are bundled
> under `robot_configs/` in the repository.

---

## Quick Links

- [Installation Guide](installation.md) — Environment setup, SSL certificates, and configuration
- [Usage Guide](usage.md) — Launching teleop, recording datasets, deferred rendering, and network modes
- [Troubleshooting](troubleshooting.md) — Common issues with detailed diagnostics and solutions

---

## Project Structure

```
telesim/
├── config/
│   ├── config.yaml                    # Global: server, Isaac Sim, cameras, recording
│   └── robots/
│       ├── openarm.yaml               # OpenArm joints, IK, cameras, teleop
│       ├── acone.yaml                 # AC One config
│       └── ffw_bg2.yaml               # FFW BG2 config (domain randomization enabled by default)
│
├── src/
│   ├── launch/                        # Per-robot entry-points
│   │   ├── openarm_teleop.py          # OpenArm Isaac Sim launcher
│   │   ├── acone_teleop.py            # AC One Isaac Sim launcher
│   │   ├── ffw_bg2_teleop.py          # FFW BG2 Isaac Sim launcher
│   │   ├── bimanual_runtime.py        # Shared control loop (all robots)
│   │   ├── webxr_bridge.py            # WebSocket ↔ ROS 2 bridge
│   │   ├── controller_provider.py     # ROS subscriber for Quest controller data
│   │   ├── control_metrics.py         # Control loop diagnostics logger
│   │   └── teleop_utils.py            # Shared launch helpers
│   ├── teleop_core/                   # Calibration, IK retargeting, smoothing, safety
│   │   ├── session.py
│   │   ├── calibration.py
│   │   ├── filters.py
│   │   ├── safety.py
│   │   └── retargeting.py
│   ├── recording/                     # Dataset pipeline
│   │   ├── deferred_renderer.py       # ★ Offline camera rendering pass
│   │   ├── lerobot_recorder.py        # IPC wrapper — enqueues frames to worker
│   │   ├── worker_process.py          # LeRobot dataset writer (runs in .venv)
│   │   ├── extract_states.py          # Reads LeRobot dataset → JSON for renderer
│   │   ├── config.py                  # RecordingConfig dataclass
│   │   ├── buttons.py                 # Controller button → recording action mapper
│   │   ├── schema.py                  # Dataset feature schema builder
│   │   ├── snapshots.py               # RecordingFrameSnapshot + diagnostics
│   │   └── ipc.py                     # Inter-process communication helpers
│   ├── robot_adapters/                # One adapter per robot
│   │   ├── base.py                    # RobotAdapter abstract base
│   │   ├── openarm.py
│   │   ├── acone.py
│   │   └── ffw_bg2.py
│   ├── isaac_backend/                 # Isaac Sim wrappers
│   │   ├── app.py                     # IsaacApp lifecycle manager
│   │   ├── camera_manager.py          # RTX camera capture (async futures)
│   │   ├── domain_randomization.py    # Object + lighting randomization
│   │   └── ros_publishers.py          # JointState + image ROS publishers
│   ├── quest_ingress/                 # WebXR message types & transport metrics
│   │   ├── message_types.py
│   │   └── metrics.py
│   └── config_loader.py               # Unified YAML config reader (CLI + API)
│
├── scripts/
│   ├── run_lan_teleop.sh              # ★ All-in-one LAN launcher (recommended)
│   ├── run_openarm_teleop.sh          # OpenArm only
│   ├── run_acone_teleop.sh            # AC One only
│   ├── run_ffw_bg2_teleop.sh          # FFW BG2 only
│   ├── run_deferred_renderer.sh       # ★ Post-teleop camera rendering pass
│   ├── run_wireless.sh                # Bridge + HTTPS server only (no Isaac Sim)
│   ├── generate_cert.sh               # SSL certificate generation
│   ├── summarize_metrics.py           # Parses control loop log → summary table
│   └── visualize_lerobot_dataset.sh   # Rerun-based dataset visualizer
│
├── web/
│   ├── webxr_streamer.html            # Quest browser WebXR controller app
│   └── https_server.py               # Minimal HTTPS file server
│
├── robot_configs/                     # Bundled USD / URDF / lula assets
├── datasets/                          # Recorded datasets (gitignored)
├── certs/                             # SSL certificates (gitignored)
└── requirements.txt
```
