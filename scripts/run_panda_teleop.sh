#!/bin/bash

# Set environment variables for Isaac Sim internal ROS 2 Bridge
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Get project root directory (parent of scripts/)
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")

# Isaac Sim installation path from env override or the shared config loader
CONFIG_FILE="$PROJECT_ROOT/config/config.yaml"
PYTHON_BIN="${PYTHON_BIN:-python3}"
if [[ -x "$PROJECT_ROOT/.venv/bin/python" ]]; then
    PYTHON_BIN="$PROJECT_ROOT/.venv/bin/python"
fi

config_value() {
    "$PYTHON_BIN" "$PROJECT_ROOT/src/config_loader.py" get --config "$CONFIG_FILE" "$1"
}

ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-$(config_value paths.isaac_sim)}"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge/humble/lib

echo "Starting Isaac Sim Teleop (USD Workflow)..."
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ISAAC_SIM_PATH: $ISAAC_SIM_PATH"

cd "$PROJECT_ROOT"
"$ISAAC_SIM_PATH/python.sh" -m src.launch.panda_teleop --config "$CONFIG_FILE" --robot panda "$@"
