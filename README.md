# Quest 3 OpenXR Streamer

A modular Python application to stream real-time controller data (pose, velocity, and inputs) from a Meta Quest 3 headset to a Linux PC using OpenXR and WiVRn.

## Features

*   **Real-time Tracking**: Captures Head, Left/Right Controller Aim, and Grip poses.
*   **Velocity Data**: Streams Linear and Angular velocity for controllers.
*   **Input Capture**: Reads Trigger, Squeeze, and Button states (A/B/X/Y, Menu, Thumbstick).
*   **ROS 2 Integration**: Publishes PoseStamped and Joy messages to ROS 2 topics.
*   **Modular Design**: Cleanly separated into graphics, core OpenXR, input management, and ROS interface modules.
*   **OpenGL Backend**: Uses a hidden GLFW window to satisfy OpenXR session requirements on Linux.

## Prerequisites

### Hardware
*   Meta Quest 3 (or compatible OpenXR headset).
*   Linux PC (tested on Ubuntu/Debian).
*   5GHz Wi-Fi router (for wireless streaming via WiVRn).

### Software
*   **ROS 2 Humble**: Installed on the system.
*   **WiVRn**: An open-source OpenXR streaming runtime.
*   **Python 3.10**: Required for ROS 2 Humble compatibility.
*   **uv**: Fast Python package installer.

## Installation

1.  **Clone the Repository**
    ```bash
    git clone https://github.com/YOUR_USERNAME/quest3-openxr-streamer.git
    cd quest3-openxr-streamer
    ```

2.  **Create a Virtual Environment (with uv)**
    We use `uv` to create a Python 3.10 environment that can access system ROS packages.
    ```bash
    uv venv .venv --python /usr/bin/python3 --system-site-packages
    source .venv/bin/activate
    ```

3.  **Install Dependencies**
    ```bash
    uv pip install -r requirements.txt
    ```

4.  **System Dependencies**
    Ensure you have GLFW and GLX libraries installed:
    ```bash
    sudo apt-get install libglfw3 libgl1-mesa-glx
    ```

## Usage

1.  **Start WiVRn Server**
    Ensure your headset is connected to WiVRn.

2.  **Source ROS 2**
    ```bash
    source /opt/ros/humble/setup.bash
    ```

3.  **Run the Streamer**
    Execute the main script.

    ```bash
    # Activate venv
    source .venv/bin/activate

    # RunWAYLAND_DISPLAY= XDG_SESSION_TYPE=x11 python quest_stream_opengl.py
    
    ```

4.  **Verify ROS Topics**
    In another terminal:
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 topic list
    ros2 topic echo /quest/right_hand/pose
    ```

## Troubleshooting

*   **"Session not focused"**: Make sure you are wearing the headset and the WiVRn application is active.
*   **"Could not load libGL.so.1"**: Install `libgl1-mesa-glx`.
*   **"Failed to init GLFW"**: Ensure you are running in a graphical environment (not a pure TTY).

## Simulation & Verification

### 1. MuJoCo Verification
We provide a lightweight MuJoCo simulation to verify the data pipeline and coordinate mapping before running heavy simulators.

**Features:**
*   Visualizes the Right Hand as a **Green Box** with axes.
*   Includes a **Dummy Robot Arm** with IK to test reachability.

**Run:**
```bash
source .venv/bin/activate
python mujoco_sim.py
```

### 2. Isaac Sim Teleoperation
Control a **Franka Panda** robot in NVIDIA Isaac Sim using the streamed data.

**Features:**
*   Uses `omni.isaac.ros2_bridge` to subscribe to ROS topics.
*   Uses `LulaKinematicsSolver` for smooth IK control.
*   Maps Quest 3 coordinate space to Robot base frame.
*   Gripper control via Controller Trigger.

**Run:**
Use the provided wrapper script to handle environment variables and launch Isaac Sim:
```bash
./run_isaac_teleop.sh
```

### 3. VPN / Remote Setup
If running the Streamer on a local laptop and Isaac Sim on a remote workstation (connected via VPN), standard ROS 2 discovery (Multicast) might fail.

**Solution:**
1.  Edit `fastdds_vpn.xml` and add the VPN IP addresses of both machines.
2.  Export the config on **BOTH** machines before running scripts:
    ```bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_vpn.xml
    ```

## License

MIT License
