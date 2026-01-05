# Quest 3 VR Teleoperation

Welcome to the documentation for the Quest 3 VR Teleoperation project.

## Purpose

This project enables real-time, low-latency teleoperation of robots using a Meta Quest 3 headset. It streams controller data (pose, buttons, triggers) from the headset to a ROS 2 environment using WebXR over WiFi.

## Key Features

- **Full Controller Tracking**: Captures 6DoF pose, trigger, grip, thumbstick, and button inputs (A/B/X/Y) from both controllers.
- **Bimanual Teleoperation**: Control dual-arm robots using both Quest controllers simultaneously.
- **Wireless WebXR**: Low-latency streaming via HTTPS over WiFi with AR passthrough.
- **Dynamic Calibration**: Works sitting or standing - your starting hand position becomes the robot's home position.
- **ROS 2 Integration**: Publishes standard `PoseStamped`, `Joy`, `JointState`, and `Image` messages.
- **Multi-Camera Support**: Head camera and wrist cameras with in-VR switching via A/X buttons.
- **LeRobot Recording Ready**: Publishes joint states and camera images for data collection.
- **Simulation Support**:
    - **OpenArm Bimanual**: Dual 7-DOF arm control with grippers.
    - **Franka Panda**: Single-arm teleoperation.

## Video Tutorial

<video src="assets/quest3_streamer.mp4" controls="controls" style="max-width: 100%;" width="100%"></video>

## Architecture Overview

The system consists of three main components:

```
┌─────────────────┐     HTTPS/WSS      ┌───────────────────┐     ROS 2 Topics     ┌────────────────────┐
│   Quest 3       │ ─────────────────► │   ROS Bridge      │ ──────────────────►  │   Isaac Sim        │
│   (WebXR App)   │    Controller      │   (Python)        │    PoseStamped       │   (Teleoperation)  │
│                 │    Pose + Inputs   │                   │    Joy, JointState   │                    │
└─────────────────┘                    └───────────────────┘                      └────────────────────┘
```

1. **WebXR Client (Quest 3)**: A web application (`web/webxr_streamer.html`) running on the headset that captures XR controller data.
2. **ROS Bridge (PC)**: A Python script (`src/webxr_ros_bridge.py`) that receives WebSocket data and publishes ROS 2 topics.
3. **Robot Control Node**: Isaac Sim scripts (`src/isaac_openarm_teleop.py`, `src/isaac_panda_teleop.py`) that subscribe to topics and control robots.

## Project Structure

```
quest3_streamer/
├── config/
│   └── config.yaml           # Centralized paths configuration
├── src/
│   ├── isaac_openarm_teleop.py   # Bimanual OpenArm control
│   ├── isaac_panda_teleop.py     # Single-arm Panda control
│   └── webxr_ros_bridge.py       # WebXR → ROS 2 bridge
├── web/
│   ├── webxr_streamer.html       # Quest browser WebXR app
│   └── https_server.py           # HTTPS server
├── scripts/
│   ├── run_openarm_teleop.sh     # Launch OpenArm teleop
│   ├── run_panda_teleop.sh       # Launch Panda teleop
│   ├── run_wireless.sh           # Launch wireless streaming
│   └── generate_cert.sh          # Generate SSL certificates
├── openarm_config/           # OpenArm robot config (USD, URDF)
├── certs/                    # SSL certificates (gitignored)
└── docs/                     # This documentation
```

## Quick Links

- [Installation Guide](installation.md) - Set up the project
- [Usage Guide](usage.md) - Run teleoperation
- [Troubleshooting](troubleshooting.md) - Common issues and solutions
