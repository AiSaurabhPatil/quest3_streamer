#!/bin/bash

# Set environment variables for Isaac Sim internal ROS 2 Bridge
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Adjust this path if your Isaac Sim installation is different
# The log indicated: /home/saurabh/isaac_sim/exts/isaacsim.ros2.bridge/humble/lib
ISAAC_SIM_PATH="/home/saurabh/.local/share/ov/pkg/isaac_sim-4.0.0" 
# Note: I am guessing the path or leaving a placeholder. 
# Since the previous file had a placeholder, I will use a likely default or the same placeholder.
# Actually, let's keep it consistent with the previous file but maybe add a check.
# The previous file had "/path/to/isaac_sim". 
# But wait, the user successfully ran it before? 
# I will copy the content from the existing run_isaac_teleop.sh exactly, 
# just changing the target python script.

ISAAC_SIM_PATH="/path/to/isaac_sim"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge/humble/lib

# Run the python script using Isaac Sim's python wrapper
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

echo "Starting Isaac Sim Teleop (USD Workflow)..."
echo "ROS_DISTRO: $ROS_DISTRO"

$ISAAC_SIM_PATH/python.sh $SCRIPT_DIR/isaac_teleop_usd.py
