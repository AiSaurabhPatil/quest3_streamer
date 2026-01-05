#!/bin/bash

# Set environment variables for Isaac Sim internal ROS 2 Bridge
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Get project root directory (parent of scripts/)
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")

# Isaac Sim installation path
ISAAC_SIM_PATH="/home/saurabh/isaac_sim"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge/humble/lib

echo "Starting OpenArm Bimanual Teleop..."
echo "ROS_DISTRO: $ROS_DISTRO"

$ISAAC_SIM_PATH/python.sh $PROJECT_ROOT/src/isaac_openarm_teleop.py
