#!/bin/bash

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")
CONFIG_FILE="$PROJECT_ROOT/config/config.yaml"
PYTHON_BIN="${PYTHON_BIN:-python3}"
if [[ -x "$PROJECT_ROOT/.venv/bin/python" ]]; then
    PYTHON_BIN="$PROJECT_ROOT/.venv/bin/python"
fi

config_value() {
    "$PYTHON_BIN" "$PROJECT_ROOT/src/config_loader.py" get --config "$CONFIG_FILE" "$1"
}

ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-$(config_value paths.isaac_sim)}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ISAAC_SIM_PATH/exts/isaacsim.ros2.core/$ROS_DISTRO/lib"

recording_requested() {
    local config_recording_enabled
    config_recording_enabled=$(config_value recording.enabled 2>/dev/null || echo false)
    if [[ "$config_recording_enabled" == "true" ]]; then
        return 0
    fi

    local arg
    for arg in "$@"; do
        case "$arg" in
            --record|--dataset-root|--dataset-root=*|--dataset-repo-id|--dataset-repo-id=*|--task|--task=*|--recording-fps|--recording-fps=*|--max-episodes|--max-episodes=*)
                return 0
                ;;
        esac
    done
    return 1
}

if recording_requested "$@"; then
    recording_python="${LEROBOT_RECORDING_PYTHON:-$PROJECT_ROOT/.venv-lerobot-v21/bin/python}"
    if [[ ! -x "$recording_python" ]]; then
        recording_python="$PROJECT_ROOT/.venv/bin/python"
    fi
    if ! "$recording_python" - <<'PY' >/dev/null
try:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
except ImportError:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
PY
    then
        echo "[Recording] LeRobot is not importable in: $recording_python"
        exit 1
    fi
fi

echo "Starting FFW BG2 Bimanual Teleop..."
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ISAAC_SIM_PATH: $ISAAC_SIM_PATH"

has_webrtc_arg() {
    local arg
    for arg in "$@"; do
        case "$arg" in
            --webrtc|--no-webrtc)
                return 0
                ;;
        esac
    done
    return 1
}

EXTRA_ARGS=()
if ! has_webrtc_arg "$@"; then
    EXTRA_ARGS+=(--webrtc)
    echo "[WebRTC] Isaac Sim streaming enabled by default. Connect with Isaac Sim WebRTC client to this host on port 49100."
fi

cd "$PROJECT_ROOT"
"$ISAAC_SIM_PATH/python.sh" -m src.launch.ffw_bg2_teleop --config "$CONFIG_FILE" --robot ffw_bg2 "${EXTRA_ARGS[@]}" "$@"
