# Usage Guide

This guide covers how to start the WebXR streamer, connect it to ROS 2, and run simulations.

## Quick Start: WebXR Streaming

The recommended way to use this project is via WebXR over a wired USB connection for minimal latency.

### 1. Establish USB Connection

1.  Connect your Meta Quest 3 to your Linux PC via USB.
2.  Enable **USB Debugging** on your Quest if you haven't already.
3.  On your PC, verify the connection:
    ```bash
    adb devices
    ```
    You should see your device ID listed.

### 2. Configure Network Proxy

To allow the Quest browser to talk to your PC's local server over USB, set up an `adb` reverse proxy. **You must run this every time you reconnect the USB cable.**

```bash
# Proxy HTTP server port
adb reverse tcp:8080 tcp:8080
# Proxy WebSocket port (for ROS bridge)
adb reverse tcp:9090 tcp:9090
```

### 3. Launch the System

You will need three terminal tabs.

**Terminal 1: HTTP Server**
Host the WebXR application.
```bash
cd /path/to/quest3-streamer
python -m http.server 8080
```

**Terminal 2: ROS Bridge**
Start the bridge that receives data from the headset and publishes ROS topics.
```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python webxr_ros_bridge.py
```

**Terminal 3: Verification**
Check if topics are being published.
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /quest/right_hand/pose
```

### 4. Start the Application on Headset

1.  Put on your Quest 3 headset.
2.  Open **Meta Quest Browser**.
3.  Navigate to `http://localhost:8080/webxr_streamer.html`.
4.  Ensure the settings are:
    -   **PC Server IP**: `localhost`
    -   **Port**: `9090`
5.  Click **"Start AR Session"**.
6.  "Allow" any permission prompts for WebXR or Passthrough.

You should now see the passthrough view, and data should be streaming to your PC.

---

## Running Simulations

### MuJoCo Verification
A lightweight simulation to verify coordinate mapping and tracking accuracy.

```bash
source .venv/bin/activate
python mujoco_sim.py
```
A green target should appear and follow your right hand's movement.

### Isaac Sim Teleoperation
Control a Franka Panda robot in NVIDIA Isaac Sim.

1.  **Launch Isaac Sim Teleop**:
    ```bash
    ./run_isaac_teleop.sh
    ```
    *Note: Do not source your system ROS setup before running this script if it conflicts with Isaac Sim's internal ROS.*

2.  **Calibration**:
    -   Sit or stand in a comfortable position.
    -   Hold your hand steady for ~1 second to trigger the dynamic calibration.
    -   The robot end-effector will snap to your hand position.

3.  **Controls**:
    -   **Move Hand**: Moves the robot end-effector.
    -   **Trigger/Grip**: Closes the gripper.

## ROS Topics Reference

| Topic | Type | Description |
|-------|------|-------------|
| `/quest/left_hand/pose` | `geometry_msgs/PoseStamped` | Position and Orientation of left controller. |
| `/quest/right_hand/pose` | `geometry_msgs/PoseStamped` | Position and Orientation of right controller. |
| `/quest/left_hand/inputs`| `sensor_msgs/Joy` | Button and axis states for left controller. |
| `/quest/right_hand/inputs`| `sensor_msgs/Joy` | Button and axis states for right controller. |

### Joy Message Mapping
- `axes[0]`: Trigger (float 0.0 - 1.0)
- `axes[1]`: Squeeze/Grip (float 0.0 - 1.0)
- `axes[2]`: Thumbstick X
- `axes[3]`: Thumbstick Y
- `buttons[0]`: A / X Button
- `buttons[1]`: B / Y Button
- `buttons[2]`: Menu Button
- `buttons[3]`: Thumbstick Click
