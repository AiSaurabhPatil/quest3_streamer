#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$PROJECT_ROOT/src"

ISAAC_SIM="${ISAAC_SIM_PATH:-/home/saurabh/isaac_sim}"

if [ ! -f "$ISAAC_SIM/python.sh" ]; then
    echo "Error: Isaac Sim python.sh not found at $ISAAC_SIM/python.sh"
    echo "Please set ISAAC_SIM_PATH to your Isaac Sim installation."
    exit 1
fi

REPO_ID=$1
OUTPUT_REPO_ID=$2

if [ -z "$REPO_ID" ] || [ -z "$OUTPUT_REPO_ID" ]; then
    echo "Usage: ./run_deferred_renderer.sh <repo-id> <output-repo-id>"
    echo "Example: ./run_deferred_renderer.sh local/quest3-openarm local/quest3-openarm-rendered"
    exit 1
fi

cd "$PROJECT_ROOT"

# ---------------------------------------------------------------------------
# Setup ROS2 environment for Isaac Sim 6.0
# ---------------------------------------------------------------------------
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ISAAC_RCLPY_PATH="$ISAAC_SIM/exts/isaacsim.ros2.core/${ROS_DISTRO}/rclpy"
ISAAC_ROS_LIB_PATH="$ISAAC_SIM/exts/isaacsim.ros2.core/${ROS_DISTRO}/lib"
if [[ -d "$ISAAC_RCLPY_PATH" ]]; then
    export PYTHONPATH="${ISAAC_RCLPY_PATH}${PYTHONPATH:+:$PYTHONPATH}"
    export LD_LIBRARY_PATH="${ISAAC_ROS_LIB_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

# ---------------------------------------------------------------------------
# Pass Virtual Environment site-packages path as argument to python script
# ---------------------------------------------------------------------------
VENV_PYTHON="${LEROBOT_RECORDING_PYTHON:-$PROJECT_ROOT/.venv/bin/python}"
SITE_PACKAGES=""
if [[ -x "$VENV_PYTHON" ]]; then
    SITE_PACKAGES=$("$VENV_PYTHON" -c "import site; print(site.getsitepackages()[0])")
fi

shift 2

echo "[Renderer] Starting Deferred Renderer for $REPO_ID -> $OUTPUT_REPO_ID"
"$ISAAC_SIM/python.sh" "$SRC_DIR/recording/deferred_renderer.py" \
    --repo-id "$REPO_ID" \
    --output-repo-id "$OUTPUT_REPO_ID" \
    --venv-python "$VENV_PYTHON" \
    "$@"
