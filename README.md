# Quest 3 VR Teleoperation

Stream real-time controller data from Meta Quest 3 to ROS 2 for robot teleoperation using WebXR.

## Features

- **Full Controller Tracking**: 6DoF pose, trigger, grip, thumbstick, buttons (A/B/X/Y)
- **Bimanual Teleoperation**: Control dual-arm robots with both Quest controllers
- **WebXR Wireless Streaming**: Low-latency via HTTPS over WiFi
- **Remote Ingress Mode**: Forward compact controller packets from a local LAN ingress to a remote Isaac server over VPN
- **ROS 2 Integration**: Publishes `PoseStamped` and `Joy` messages
- **Isaac Sim Ready**: Supports OpenArm bimanual  and Franka Panda
- **Multi-Camera Support**: Head and wrist cameras with switchable views
- **Centralized Config**: All paths in `config/config.yaml`

## Video Preview

![Quest 3 Streamer Demo](docs/assets/quest3_streamer_preview.gif)

> **Note**: Full quality video in `tutorial_video/quest3_streamer.mp4`

## Project Structure

```
quest3_streamer/
├── config/
│   └── config.yaml           # Centralized paths configuration
├── src/
│   ├── isaac_openarm_teleop.py   # Bimanual OpenArm teleoperation
│   ├── isaac_panda_teleop.py     # Franka Panda teleoperation
│   └── webxr_ros_bridge.py       # WebXR → ROS 2 bridge
├── web/
│   ├── webxr_streamer.html       # Quest browser WebXR app
│   └── https_server.py           # HTTPS server for wireless
├── scripts/
│   ├── run_openarm_teleop.sh     # Launch OpenArm teleop
│   ├── run_panda_teleop.sh       # Launch Panda teleop
│   ├── run_wireless.sh           # Launch wireless streaming
│   └── generate_cert.sh          # Generate SSL certificates
├── robot_configs/openarm_config/           # OpenArm robot config (USD, URDF)
├── certs/                    # SSL certificates (gitignored)
└── docs/                     # Documentation
```

## Quick Start

### Prerequisites

- Meta Quest 3 on same WiFi network as PC
- Linux PC with ROS 2 Humble (Ubuntu 22.04)
- Python 3.10+
- NVIDIA Isaac Sim (for robot simulation)

### Installation

```bash
git clone https://github.com/AiSaurabhPatil/quest3_streamer.git
cd quest3_streamer

# Create virtual environment
python3 -m venv .venv --system-site-packages
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

## Usage

### Wireless Streaming

One-command setup:

```bash
./scripts/run_wireless.sh
```

This starts both HTTPS server and ROS bridge. On Quest browser:
1. Navigate to `https://<YOUR_PC_IP>:8000/web/webxr_streamer.html`
2. Accept security warning (self-signed cert)
3. Click **"Start AR Session"**

### Remote Isaac Server Over VPN

Run a local ingress near the Quest:

```bash
python src/webxr_ros_bridge.py \
  --mode ingress \
  --host 0.0.0.0 \
  --port 9999 \
  --cert certs/cert.pem \
  --key certs/key.pem \
  --forward-url ws://<REMOTE_SERVER_IP>:9998
```

Run the remote receiver near Isaac Sim:

```bash
python src/webxr_ros_bridge.py --mode remote-receiver --host 0.0.0.0 --port 9998
```

The bridge now logs compact transport metrics such as packet rate, jitter, end-to-end packet age, and VPN hop age.

### Isaac Sim Teleoperation

#### OpenArm Bimanual Robot

```bash
./scripts/run_openarm_teleop.sh
```

**Features:**
- Both controllers mapped to left/right arms
- Dynamic calibration (works sitting or standing)
- Full 6DoF orientation tracking  
- Smooth gripper control (trigger/grip)
- Camera switching with A/X buttons
- Publishes `/joint_states` and camera images for LeRobot

**Calibration:**
1. Start the teleop script and wait for Isaac Sim to load
2. Put on your Quest headset and start the AR session
3. **Hold both controllers steady** in a comfortable position for ~1 second
4. The script will print "CALIBRATION COMPLETE" for each arm
5. Your current hand position becomes the robot's home position
6. Move your hands to control the robot!

> **Tip**: Stand or sit in a comfortable pose during calibration. The robot will mirror your hand movements relative to this starting position.

**Controls:**
| Controller | Action |
|------------|--------|
| Left Controller | Controls left arm |
| Right Controller | Controls right arm |
| Trigger/Grip | Close gripper |
| A/X Button | Cycle camera views |

#### Franka Panda Robot

```bash
./scripts/run_panda_teleop.sh
```

Single-arm teleoperation with right controller.

## Configuration

Edit `config/config.yaml` to customize paths:

```yaml
paths:
  isaac_sim: "/path/to/isaac_sim"
  openarm:
    usd: "robot_configs/openarm_config/openarm_bimanual/openarm_bimanual_env.usd"
    urdf: "robot_configs/openarm_config/openarm_bimanual_stl.urdf"
  panda:
    usd: "environment.usd"
  certs:
    cert: "certs/cert.pem"
    key: "certs/key.pem"

server:
  host: "0.0.0.0"
  websocket_port: 9999
  https_port: 8000

teleop:
  deadman_timeout_ms: 250
  hard_timeout_ms: 1000
```

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/quest/left_hand/pose` | `PoseStamped` | Left controller 6DoF pose |
| `/quest/right_hand/pose` | `PoseStamped` | Right controller 6DoF pose |
| `/quest/left_hand/inputs` | `Joy` | Left controller buttons/axes |
| `/quest/right_hand/inputs` | `Joy` | Right controller buttons/axes |
| `/joint_states` | `JointState` | Robot joint positions (for recording) |
| `/camera/head/image_raw` | `Image` | Head camera (OpenArm) |
| `/camera/wrist_left/image_raw` | `Image` | Left wrist camera |
| `/camera/wrist_right/image_raw` | `Image` | Right wrist camera |


## Troubleshooting

| Issue | Solution |
|-------|----------|
| 404 error on webpage | Use `/web/webxr_streamer.html` path |
| WebSocket disconnected | Check PC IP is correct |
| WebXR Not Available | Must use HTTPS for wireless |
| IK failures | Move hand to reachable position |
| No controller data | Run `ros2 topic list \| grep quest` |

## License

MIT License
