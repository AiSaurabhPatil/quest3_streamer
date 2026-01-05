# Usage Guide

Complete guide to running VR teleoperation with Quest 3.

## Quick Start: Wireless Streaming

The simplest way to get started:

```bash
./scripts/run_wireless.sh
```

This single command:

1. Generates SSL certificates if missing
2. Starts HTTPS server on port 8000
3. Starts WebSocket ROS bridge on port 9090
4. Prints the URL for your Quest browser

### Connect From Quest 3

1. Put on your Quest 3 headset
2. Open **Meta Quest Browser**
3. Navigate to the URL shown in terminal: `https://<YOUR_PC_IP>:8000/web/webxr_streamer.html`
4. **Accept the security warning** (click "Advanced" → "Proceed")
5. Enter your PC's IP address in the form
6. Click **"Start AR Session"**
7. Allow any permission prompts

You should see the AR passthrough view and controller tracking status.

### Verify ROS Topics

In a new terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep quest

# Should show:
# /quest/left_hand/pose
# /quest/right_hand/pose
# /quest/left_hand/inputs
# /quest/right_hand/inputs
```

Echo a topic to see live data:

```bash
ros2 topic echo /quest/right_hand/pose
```

---

## OpenArm Bimanual Teleoperation

Control a dual-arm OpenArm robot in Isaac Sim.

### Launch

```bash
./scripts/run_openarm_teleop.sh
```

### Calibration Process

1. **Wait for Isaac Sim to load** - The script will print initialization progress
2. **Start AR session on Quest** - As described above
3. **Hold both controllers steady** in a comfortable position
4. **Wait ~1 second** - The system collects 30 samples for calibration
5. **Look for "CALIBRATION COMPLETE"** message in terminal
6. **Start moving!** - Your hands now control the robot arms

!!! tip "Calibration Tips"
    - Stand or sit in a relaxed, natural pose
    - Your calibration position becomes the robot's "home" position
    - All movements are relative to this starting point
    - You can recalibrate by restarting the script

### Controls

| Input | Action |
|-------|--------|
| Left Controller Movement | Moves left arm end-effector |
| Right Controller Movement | Moves right arm end-effector |
| Left Trigger OR Grip | Closes left gripper |
| Right Trigger OR Grip | Closes right gripper |
| A Button (Right) | Cycle camera view |
| X Button (Left) | Cycle camera view |

### Camera Views

The A/X buttons cycle through available cameras:

1. **Perspective** - Default third-person view
2. **Head Camera** - Robot's head-mounted camera
3. **Left Wrist Camera** - Left arm's wrist camera
4. **Right Wrist Camera** - Right arm's wrist camera

### Data Recording (LeRobot)

The OpenArm teleop script publishes data for recording:

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `JointState` | All robot joint positions |
| `/camera/head/image_raw` | `Image` | Head camera feed |
| `/camera/wrist_left/image_raw` | `Image` | Left wrist camera |
| `/camera/wrist_right/image_raw` | `Image` | Right wrist camera |

Camera images are published asynchronously at ~15 Hz to avoid performance impact.

---

## Franka Panda Teleoperation

Control a single Franka Panda arm.

### Launch

```bash
./scripts/run_panda_teleop.sh
```

### Controls

| Input | Action |
|-------|--------|
| Right Controller Movement | Moves end-effector |
| Trigger OR Grip | Closes gripper |
| A Button | Switch camera view |

### Calibration

Same as OpenArm - hold your right hand steady for ~1 second after starting.

---

## ROS Topics Reference

### Controller Topics (from Quest)

| Topic | Type | Description |
|-------|------|-------------|
| `/quest/left_hand/pose` | `geometry_msgs/PoseStamped` | Left controller 6DoF pose |
| `/quest/right_hand/pose` | `geometry_msgs/PoseStamped` | Right controller 6DoF pose |
| `/quest/left_hand/inputs` | `sensor_msgs/Joy` | Left controller buttons/axes |
| `/quest/right_hand/inputs` | `sensor_msgs/Joy` | Right controller buttons/axes |

### Joy Message Mapping

```python
# Axes (float values)
axes[0] = trigger      # 0.0 to 1.0
axes[1] = squeeze/grip # 0.0 to 1.0
axes[2] = thumbstick_x # -1.0 to 1.0
axes[3] = thumbstick_y # -1.0 to 1.0

# Buttons (0 or 1)
buttons[0] = A / X button
buttons[1] = B / Y button
buttons[2] = Menu button
buttons[3] = Thumbstick click
```

### Robot Topics (from Isaac Sim)

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Robot joint positions |
| `/camera/head/image_raw` | `sensor_msgs/Image` | Head camera (480x360 RGB) |
| `/camera/wrist_left/image_raw` | `sensor_msgs/Image` | Left wrist camera |
| `/camera/wrist_right/image_raw` | `sensor_msgs/Image` | Right wrist camera |

---

## Advanced Configuration

### Teleoperation Parameters

Edit the `CONFIG` dictionary in `src/isaac_openarm_teleop.py`:

```python
CONFIG = {
    "pos_scale": 1.0,          # VR to robot movement scale (1.0 = 1:1)
    "smoothing": 0.9,          # Motion smoothing (0=none, 0.9=very smooth)
    "gripper_threshold": 0.5,  # Trigger threshold to close gripper
    "gripper_speed": 0.05,     # Gripper movement speed per frame
    "calibration_samples": 30, # Samples for calibration (~1 second)
    "debug_ik": False,         # Enable IK debug output
}
```

### Server Ports

Edit `config/config.yaml`:

```yaml
server:
  websocket_port: 9090  # WebSocket for controller data
  https_port: 8000      # HTTPS for WebXR page
```

### IK Configuration

Each arm has its own IK configuration in `openarm_config/left_arm/` and `openarm_config/right_arm/`:

- `robot_descriptor.yaml` - Lula IK configuration
- Joint limits, end-effector frame, etc.

---

## Next Steps

- See [Troubleshooting](troubleshooting.md) if you encounter issues
