# Installation Guide

## Prerequisites

Before setting up the project, ensure you have the following hardware and software:

### Hardware
- **Meta Quest 3** headset.
- **USB-C Data Cable** (high quality recommended for low latency).
- **Linux PC** (tested on Ubuntu 22.04).

### Software
- **ROS 2 Humble**: [Installation Guide](https://docs.ros.org/en/humble/Installation.html).
- **Python 3.10+**.
- **ADB (Android Debug Bridge)**: For USB communication with the headset.
- **Google Chrome / Edge** (on PC for debugging) and **Meta Quest Browser** (on headset).

## Step-by-Step Installation

### 1. Clone the Repository

```bash
git clone https://github.com/AiSaurabhPatil/quest3-streamer.git
cd quest3-streamer
```

### 2. Set Up Virtual Environment

It is recommended to use `uv` or `venv` to manage python dependencies and avoid conflicts with system packages.

```bash
# Using uv (recommended)
uv venv .venv --python /usr/bin/python3 --system-site-packages
source .venv/bin/activate

# OR using standard venv
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
```

### 3. Install Python Dependencies

```bash
# Using uv
uv pip install -r requirements.txt

# OR using pip
pip install -r requirements.txt
```

### 4. Install System Dependencies

ADB is required for USB communication.

```bash
sudo apt-get install android-tools-adb
```

## Verify Installation

To check if everything is installed correctly, you can run the MuJoCo simulation test:

```bash
source .venv/bin/activate
python mujoco_sim.py
```

If a window opens showing a robot arm or a target following your mouse/input (if connected), the environment is set up correctly.
