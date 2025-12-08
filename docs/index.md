# Quest 3 VR Teleoperation

Welcome to the documentation for the Quest 3 VR Teleoperation project.

## Purpose

The primary goal of this codebase is to enable real-time, low-latency teleoperation of robots (such as the Franka Panda) using a Meta Quest 3 headset. It streams controller data (pose, buttons, triggers) from the headset to a ROS 2 environment using WebXR.

## Key Features

- **Full Controller Tracking**: Captures 6DoF pose, trigger, grip, thumbstick, and button inputs (A/B/X/Y).
- **WebXR + USB**: Uses WebXR for broad compatibility and USB tethering for low-latency (~2-5ms) streaming.
- **AR Passthrough**: Allows operators to see their physical surroundings while controlling the robot, enhancing safety and situational awareness.
- **ROS 2 Integration**: Seamlessly integrates with the ROS 2 ecosystem by publishing standard `PoseStamped` and `Joy` messages.
- **Simulation Support**: Ready-to-use integration with:
    - **Isaac Sim**: High-fidelity simulation with dynamic calibration.
    - **MuJoCo**: Lightweight physics simulation for quick verification.

## Architecture Overview

The system consists of three main components:
1.  **WebXR Client (Quest 3)**: A web application running on the headset that captures XR input data.
2.  **ROS Bridge (PC)**: A Python script (`webxr_ros_bridge.py`) that acts as a WebSocket server, receiving data from the headset and publishing it as ROS 2 topics.
3.  **Robot Control Node**: Nodes that subscribe to the ROS topics to control physical or simulated robots (e.g., `isaac_teleop.py`).
