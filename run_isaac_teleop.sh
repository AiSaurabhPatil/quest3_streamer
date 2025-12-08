#!/bin/bash

# Set environment variables for Isaac Sim internal ROS 2 Bridge
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Adjust this path if your Isaac Sim installation is different
# The log indicated: /home/saurabh/isaac_sim/exts/isaacsim.ros2.bridge/humble/lib
ISAAC_SIM_PATH="/path/to/isaac_sim"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge/humble/lib

# Run the python script using Isaac Sim's python wrapper
# We assume this script is in the same directory as isaac_teleop.py
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

echo "Starting Isaac Sim Teleop..."
echo "ROS_DISTRO: $ROS_DISTRO"
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"

$ISAAC_SIM_PATH/python.sh $SCRIPT_DIR/isaac_teleop.py

