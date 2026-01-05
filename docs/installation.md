# Installation Guide

Complete guide to set up the Quest 3 VR Teleoperation project.

## Prerequisites

### Hardware

| Component | Description |
|-----------|-------------|
| **Meta Quest 3** | VR headset with controllers |
| **Linux PC** | Ubuntu 22.04 recommended |
| **WiFi Network** | Quest and PC on same network |
| **NVIDIA GPU** | Required for Isaac Sim |

### Software

| Software | Version | Purpose |
|----------|---------|---------|
| ROS 2 Humble | Latest | Robot communication |
| Python | 3.10+ | Bridge and teleop scripts |
| NVIDIA Isaac Sim | 5.0.0+ | Robot simulation |
| Meta Quest Browser | Latest | WebXR client |

## Step-by-Step Installation

### 1. Clone the Repository

```bash
git clone https://github.com/AiSaurabhPatil/quest3_streamer.git
cd quest3_streamer
```

### 2. Set Up Virtual Environment

Create a Python virtual environment with ROS 2 access:

```bash
# Create venv with system site packages (for ROS access)
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
```

### 3. Install Python Dependencies

```bash
pip install -r requirements.txt
```

**Dependencies installed:**

| Package | Purpose |
|---------|---------|
| `numpy` | Numerical computations |
| `scipy` | Rotation transformations |
| `websockets` | WebSocket server |
| `pyyaml` | Config file parsing |

### 4. Configure Isaac Sim Path

Edit `config/config.yaml` to set your Isaac Sim installation path:

```yaml
paths:
  isaac_sim: "/path/to/your/isaac_sim"  # Update this!
```

### 5. Generate SSL Certificates

WebXR requires HTTPS for wireless connections. Generate self-signed certificates:

```bash
./scripts/generate_cert.sh
```

This creates `certs/cert.pem` and `certs/key.pem`.

## Verify Installation

### Test 1: ROS Bridge

```bash
source .venv/bin/activate
python src/webxr_ros_bridge.py --help
```

Should show available command-line options.

### Test 2: Wireless Streaming

```bash
./scripts/run_wireless.sh
```

Should print:
- HTTPS server URL
- WebSocket server URL
- Your PC's IP address

### Test 3: Isaac Sim (Optional)

If you have Isaac Sim installed:

```bash
./scripts/run_openarm_teleop.sh
```

Isaac Sim should launch and load the OpenArm robot.

## Configuration Reference

The `config/config.yaml` file centralizes all paths:

```yaml
paths:
  # Isaac Sim installation (absolute path)
  isaac_sim: "/home/user/isaac_sim"
  
  # OpenArm robot configuration
  openarm:
    usd: "openarm_config/openarm_bimanual/openarm_bimanual.usd"
    urdf: "openarm_config/openarm_bimanual_stl.urdf"
    left_arm_config: "openarm_config/left_arm"
    right_arm_config: "openarm_config/right_arm"
  
  # Panda robot configuration
  panda:
    usd: "environment.usd"
  
  # SSL certificates
  certs:
    cert: "certs/cert.pem"
    key: "certs/key.pem"

# Server configuration
server:
  websocket_port: 9090
  https_port: 8000
```

## Next Steps

Once installed, proceed to the [Usage Guide](usage.md) to start teleoperating robots.
