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
export ISAAC_SIM_PATH
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge/humble/lib"

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

python_version_line() {
    "$1" - <<'PY'
import sys
print(f"{sys.executable} (Python {sys.version.split()[0]})")
PY
}

ensure_lerobot_available_to_recording_python() {
    local recording_python="${LEROBOT_RECORDING_PYTHON:-}"
    if [[ -z "$recording_python" ]]; then
        local dataset_format
        dataset_format=$(config_value recording.dataset_format 2>/dev/null || echo auto)
        if [[ "$dataset_format" == "v2.1" && -x "$PROJECT_ROOT/.venv-lerobot-v21/bin/python" ]]; then
            recording_python="$PROJECT_ROOT/.venv-lerobot-v21/bin/python"
        else
            recording_python="$PROJECT_ROOT/.venv/bin/python"
        fi
    fi
    if [[ ! -x "$recording_python" ]]; then
        echo "[Recording] LeRobot worker Python was not found or is not executable:"
        echo "  $recording_python"
        echo "[Recording] Create the project .venv or set LEROBOT_RECORDING_PYTHON to a Python that has LeRobot installed."
        return 1
    fi

    if "$recording_python" - <<'PY' >/dev/null
try:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
except ImportError:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
PY
    then
        echo "[Recording] LeRobot writer will run outside Isaac Sim using:"
        python_version_line "$recording_python"
        return 0
    fi

    echo "[Recording] LeRobot is not importable in the recording worker Python:"
    python_version_line "$recording_python"
    echo "[Recording] Install LeRobot in that environment, not in Isaac Sim's Python:"
    echo "  \"$recording_python\" -m pip install \"lerobot==0.3.2\"  # dataset v2.1"
    echo "  \"$recording_python\" -m pip install \"lerobot>=0.4.0\"  # dataset v3.0"
    return 1
}

if recording_requested "$@"; then
    ensure_lerobot_available_to_recording_python || exit 1
fi

echo "Starting AC One Bimanual Teleop..."
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
"$ISAAC_SIM_PATH/python.sh" -m src.launch.acone_teleop --config "$CONFIG_FILE" --robot acone "${EXTRA_ARGS[@]}" "$@"
