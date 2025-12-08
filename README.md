# Quest 3 VR Teleoperation

Stream real-time controller data (pose, buttons, triggers) from Meta Quest 3 to ROS 2 for robot teleoperation using WebXR.

## Features

- **Full Controller Tracking**: 6DoF pose, trigger, grip, thumbstick, buttons (A/B/X/Y)
- **WebXR + USB**: Low-latency streaming via USB with AR passthrough
- **ROS 2 Integration**: Publishes `PoseStamped` and `Joy` messages
- **Isaac Sim Ready**: Includes Franka Panda teleoperation with dynamic calibration
- **MuJoCo Verification**: Test coordinate mapping before Isaac Sim

## Quick Start

### Prerequisites

- Meta Quest 3 with USB cable
- Linux PC with ROS 2 Humble (tested on Ubuntu 22.04)
- Python 3.10+, ADB installed

### Installation

```bash
git clone https://github.com/AiSaurabhPatil/quest3-streamer.git
cd quest3-streamer

# Create virtual environment with ROS access
uv venv .venv --python /usr/bin/python3 --system-site-packages
source .venv/bin/activate

# Install dependencies
uv pip install -r requirements.txt

# System dependencies
sudo apt-get install android-tools-adb
```

## Usage: WebXR Streaming

WebXR runs natively on Quest and captures **all** controller inputs reliably.

### Low-Latency USB Connection

Connect Quest via USB for minimal latency (~2-5ms):

```bash
# Setup ADB reverse proxy (run once after connecting USB)
adb reverse tcp:8080 tcp:8080  # HTTP server
adb reverse tcp:9090 tcp:9090  # WebSocket

# Verify connection
adb devices
```

### Start Streaming

**Terminal 1** - HTTP Server:
```bash
cd /path/to/quest3-streamer
python -m http.server 8080
```

**Terminal 2** - ROS Bridge:
```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python webxr_ros_bridge.py
```

**On Quest Browser**:
1. Open: `http://localhost:8080/webxr_streamer.html`
2. Set PC Server IP to: `localhost`
3. Set Port to: `9090`
4. Click **"Start AR Session"**

You'll see AR passthrough with your controllers being tracked.

**Terminal 3** - Verify:
```bash
ros2 topic echo /quest/right_hand/pose
ros2 topic echo /quest/right_hand/inputs
```

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/quest/left_hand/pose` | `PoseStamped` | Left controller 6DoF pose |
| `/quest/right_hand/pose` | `PoseStamped` | Right controller 6DoF pose |
| `/quest/left_hand/inputs` | `Joy` | Left controller buttons/axes |
| `/quest/right_hand/inputs` | `Joy` | Right controller buttons/axes |

### Joy Message Format

```
axes[0] = trigger (0.0-1.0)
axes[1] = squeeze/grip (0.0-1.0)
axes[2] = thumbstick X (-1.0 to 1.0)
axes[3] = thumbstick Y (-1.0 to 1.0)
buttons[0] = A/X button
buttons[1] = B/Y button
buttons[2] = Menu
buttons[3] = Thumbstick click
```

## Simulation & Verification

### 1. MuJoCo Verification

Test the data pipeline before Isaac Sim:

```bash
source .venv/bin/activate
python mujoco_sim.py
```

Green target follows your right hand.

### 2. Isaac Sim Teleoperation

Control a **Franka Panda** robot with your Quest controller.

**Features:**
- Dynamic calibration (works sitting or standing)
- Full 6DoF orientation tracking
- Gripper control via trigger/grip button
- Automatic workspace clamping

**Run** (do NOT source system ROS):
```bash
./run_isaac_teleop.sh
```

**Usage:**
1. Hold hand steady for ~1 second (calibration)
2. Move hand to control robot end-effector
3. Press trigger or grip to close gripper


## Troubleshooting

| Issue | Solution |
|-------|----------|
| `adb devices` shows nothing | Enable USB debugging on Quest, try different cable |
| WebSocket disconnected | Check IP is `localhost` when using USB |
| WebXR Not Available | Use `http://localhost:...` not IP address |
| IK failures in Isaac Sim | Move hand to reachable position, check workspace limits |
| Black screen in AR | Refresh page, restart AR session |

## File Structure

```
├── webxr_streamer.html     # Quest browser app (WebXR)
├── webxr_ros_bridge.py     # WebSocket → ROS bridge
├── isaac_teleop.py         # Isaac Sim Franka control
├── mujoco_sim.py           # MuJoCo verification
└── run_isaac_teleop.sh     # Isaac Sim launcher
```

## License

MIT License
