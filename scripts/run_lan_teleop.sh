#!/bin/bash
# =============================================================================
#  run_lan_teleop.sh — Local LAN Teleop (Quest headset on same WiFi network)
# =============================================================================
# Use this when your Quest headset and workstation are on the SAME local
# network. No VPN or packet forwarding is needed. The Quest connects directly
# to this machine's LAN IP.
#
# What this script does:
#   1. Starts the HTTPS server (serves the WebXR page to the Quest browser)
#   2. Starts the WebSocket bridge in "direct" mode (receives controller data
#      and publishes ROS topics using Isaac Sim's bundled rclpy)
#   3. Starts the robot teleop (acone by default; pass --robot openarm etc.)
#      with the WINDOWED GUI by default (view the sim on this workstation's
#      monitor). Low latency comes from render throttling, not headless mode.
#      Pass --webrtc to instead stream to a client on another device.
#
# Usage:
#   ./scripts/run_lan_teleop.sh [--robot acone|openarm|panda] [extra teleop args]
#
# Example (with recording):
#   ./scripts/run_lan_teleop.sh --robot acone --record --task "pick and place"
# Example (stream to a WebRTC client on another device instead of local GUI):
#   ./scripts/run_lan_teleop.sh --robot acone --webrtc
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Resolve paths
# ---------------------------------------------------------------------------
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")
cd "$PROJECT_ROOT"

CONFIG_FILE="$PROJECT_ROOT/config/config.yaml"
PYTHON_BIN="${PYTHON_BIN:-python3}"
if [[ -x "$PROJECT_ROOT/.venv/bin/python" ]]; then
    PYTHON_BIN="$PROJECT_ROOT/.venv/bin/python"
fi

# ---------------------------------------------------------------------------
# Helper: read a value from config.yaml via config_loader.py
# ---------------------------------------------------------------------------
config_value() {
    "$PYTHON_BIN" "$PROJECT_ROOT/src/config_loader.py" get --config "$CONFIG_FILE" "$1" 2>/dev/null || echo ""
}

# ---------------------------------------------------------------------------
# Parse our own arguments (everything else is forwarded to the teleop script)
# ---------------------------------------------------------------------------
ROBOT="${ROBOT:-acone}"
TELEOP_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --robot)
            ROBOT="$2"
            shift 2
            ;;
        --robot=*)
            ROBOT="${1#*=}"
            shift
            ;;
        *)
            TELEOP_ARGS+=("$1")
            shift
            ;;
    esac
done

# For local LAN low-latency teleop on the workstation's own monitor, use the
# WINDOWED GUI (--no-webrtc). Since you view the sim directly on this PC's
# screen, direct rendering to the window is lower visual latency than encoding
# to WebRTC and decoding in a client on the same machine. The control-loop
# speedup instead comes from render throttling (isaac.render_every_n_steps in
# config.yaml), which renders only every Nth physics step regardless of mode.
# The per-robot scripts default to --webrtc, so we inject --no-webrtc unless the
# user explicitly chose otherwise.
#   To view the sim on a DIFFERENT device over the network, pass --webrtc
#   (requires the Isaac Sim WebRTC client pointing at this host, port 49100).
_has_webrtc_flag=false
for _arg in "${TELEOP_ARGS[@]+"${TELEOP_ARGS[@]}"}"; do
    if [[ "$_arg" == "--webrtc" || "$_arg" == "--no-webrtc" ]]; then
        _has_webrtc_flag=true
        break
    fi
done
if [[ "$_has_webrtc_flag" == "false" ]]; then
    TELEOP_ARGS=("--no-webrtc" "${TELEOP_ARGS[@]+"${TELEOP_ARGS[@]}"}")
fi
unset _has_webrtc_flag _arg

# ---------------------------------------------------------------------------
# Read config values
# ---------------------------------------------------------------------------
PORT_BRIDGE=$(config_value server.websocket_port)
PORT_BRIDGE="${PORT_BRIDGE:-9999}"

PORT_HTTPS=$(config_value server.https_port)
PORT_HTTPS="${PORT_HTTPS:-8000}"

CERT_PATH=$(config_value paths.certs.cert)
CERT_PATH="${CERT_PATH:-certs/cert.pem}"

KEY_PATH=$(config_value paths.certs.key)
KEY_PATH="${KEY_PATH:-certs/key.pem}"

# ---------------------------------------------------------------------------
# Function to free up ports if they are already in use
# ---------------------------------------------------------------------------
free_port() {
    local port="$1"
    if command -v lsof >/dev/null 2>&1; then
        local pids
        pids=$(lsof -t -i:"$port" -sTCP:LISTEN 2>/dev/null || true)
        if [[ -n "$pids" ]]; then
            echo "Port $port is in use by PID(s): $pids. Freeing it up..."
            kill -9 $pids 2>/dev/null || true
            sleep 0.5
        fi
    elif command -v fuser >/dev/null 2>&1; then
        echo "Port $port is in use. Freeing it up..."
        fuser -k -n tcp "$port" >/dev/null 2>&1 || true
        sleep 0.5
    else
        local pid
        pid=$(ss -lptn "sport = :$port" 2>/dev/null | grep -oP 'pid=\K\d+' || true)
        if [[ -n "$pid" ]]; then
            echo "Port $port is in use by PID: $pid. Freeing it up..."
            kill -9 "$pid" 2>/dev/null || true
            sleep 0.5
        fi
    fi
}

free_port "$PORT_BRIDGE"
free_port "$PORT_HTTPS"


# ---------------------------------------------------------------------------
# Locate Isaac Sim and its bundled rclpy
# Isaac Sim 6.0 bundles rclpy under exts/isaacsim.ros2.core/jazzy/rclpy (and humble/rclpy)
# We need both PYTHONPATH and LD_LIBRARY_PATH set so the bridge can use rclpy
# without requiring a system ROS installation.
# ---------------------------------------------------------------------------
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-$(config_value paths.isaac_sim)}"
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-/home/saurabh/isaac_sim}"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

ISAAC_RCLPY_PATH="$ISAAC_SIM_PATH/exts/isaacsim.ros2.core/${ROS_DISTRO}/rclpy"
ISAAC_ROS_LIB_PATH="$ISAAC_SIM_PATH/exts/isaacsim.ros2.core/${ROS_DISTRO}/lib"

if [[ -d "$ISAAC_RCLPY_PATH" ]]; then
    export PYTHONPATH="${ISAAC_RCLPY_PATH}${PYTHONPATH:+:$PYTHONPATH}"
    export LD_LIBRARY_PATH="${ISAAC_ROS_LIB_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    echo "Using bundled rclpy from: $ISAAC_RCLPY_PATH"
else
    echo "WARNING: Isaac Sim bundled rclpy not found at: $ISAAC_RCLPY_PATH"
    echo "         The WebSocket bridge may fail if rclpy is not installed system-wide."
fi

# ---------------------------------------------------------------------------
# Determine local LAN IP (used to print the Quest URL)
# ---------------------------------------------------------------------------
LAN_IP=$(hostname -I | awk '{print $1}')

# ---------------------------------------------------------------------------
# Cleanup: kill all background processes on exit (EXIT trap handles all exit cases)
# ---------------------------------------------------------------------------
HTTPS_PID=""
BRIDGE_PID=""

cleanup() {
    # Disable the trap to prevent recursion
    trap - EXIT INT TERM
    echo ""
    echo "Shutting down LAN teleop..."
    _kill_and_wait "$BRIDGE_PID" "WebSocket bridge"
    _kill_and_wait "$HTTPS_PID" "HTTPS server"
}

_kill_and_wait() {
    local pid="$1"
    local label="${2:-process}"
    [[ -z "$pid" ]] && return
    # Try SIGTERM first, give it 3 seconds, then SIGKILL as fallback.
    kill "$pid" 2>/dev/null || return
    local i=0
    while kill -0 "$pid" 2>/dev/null && (( i < 6 )); do
        sleep 0.5
        (( i++ ))
    done
    if kill -0 "$pid" 2>/dev/null; then
        echo "  $label (PID $pid) did not exit — sending SIGKILL"
        kill -9 "$pid" 2>/dev/null || true
    fi
}
# Trap EXIT, INT (Ctrl+C), and TERM (kill)
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# Activate virtual environment
# ---------------------------------------------------------------------------
# shellcheck source=/dev/null
source "$PROJECT_ROOT/.venv/bin/activate"

# ---------------------------------------------------------------------------
# Generate SSL certificates if missing
# (WebXR requires HTTPS; the Quest must accept the self-signed cert once)
# ---------------------------------------------------------------------------
if [[ ! -f "$CERT_PATH" || ! -f "$KEY_PATH" ]]; then
    echo "Certificates not found. Generating them now for LAN IP: $LAN_IP"
    bash "$PROJECT_ROOT/scripts/generate_cert.sh" --ip "$LAN_IP"
fi

# ---------------------------------------------------------------------------
# Print banner
# ---------------------------------------------------------------------------
echo ""
echo "======================================================================"
echo "  Quest 3 -- Local LAN Teleop"
echo "======================================================================"
echo "  Mode   : direct (Quest connects directly to THIS workstation)"
echo "  Robot  : $ROBOT"
echo "  Render : windowed GUI on this workstation (lowest visual latency)"
echo "           Pass --webrtc to stream to a client on another device instead."
echo ""
echo "  Quest URL (open in Quest browser):"
echo "    https://$LAN_IP:$PORT_HTTPS/web/webxr_streamer.html"
echo ""
echo "  WebSocket endpoint: wss://$LAN_IP:$PORT_BRIDGE"
echo "======================================================================"
echo ""
echo "  Steps:"
echo "  1. On your Quest, open the browser and navigate to the URL above."
echo "  2. Accept the security warning (self-signed certificate)."
echo "  3. Tap 'Start AR Session' to begin streaming controller data."
echo "  4. View the sim in the Isaac Sim window on this workstation's monitor."
echo ""

# ---------------------------------------------------------------------------
# Step 1: Start HTTPS server (background)
# Serves the WebXR HTML page to the Quest browser over HTTPS.
# ---------------------------------------------------------------------------
echo "Starting HTTPS server on port $PORT_HTTPS..."
"$PYTHON_BIN" "$PROJECT_ROOT/web/https_server.py" "$PORT_HTTPS" \
    --cert "$CERT_PATH" --key "$KEY_PATH" \
    >/dev/null 2>&1 &
HTTPS_PID=$!
echo "  PID: $HTTPS_PID"

sleep 0.5

# ---------------------------------------------------------------------------
# Step 2: Start WebSocket bridge in "direct" mode (background)
# Receives controller packets from the Quest and publishes ROS topics locally.
# Uses Isaac Sim's bundled rclpy (set in PYTHONPATH above).
# ---------------------------------------------------------------------------
echo "Starting WebSocket bridge (direct mode) on port $PORT_BRIDGE..."
"$PYTHON_BIN" -m src.launch.webxr_bridge \
    --mode direct \
    --config "$CONFIG_FILE" \
    --host 0.0.0.0 \
    --port "$PORT_BRIDGE" \
    --cert "$CERT_PATH" \
    --key "$KEY_PATH" \
    &
BRIDGE_PID=$!
echo "  PID: $BRIDGE_PID"

sleep 1

# ---------------------------------------------------------------------------
# Step 3: Start robot teleop (foreground -- keeps the terminal alive)
# Isaac Sim + ROS run here, consuming ROS topics published by the bridge above.
# ---------------------------------------------------------------------------
echo ""
echo "Starting $ROBOT teleop (Isaac Sim launching -- this may take a moment)..."
echo ""

TELEOP_SCRIPT="$PROJECT_ROOT/scripts/run_${ROBOT}_teleop.sh"
if [[ ! -f "$TELEOP_SCRIPT" ]]; then
    echo "ERROR: No teleop script found for robot '$ROBOT': $TELEOP_SCRIPT"
    echo "  Available options: acone, openarm, panda"
    cleanup
fi

# Run teleop in the foreground; when it exits, cleanup() kills the bg procs
bash "$TELEOP_SCRIPT" "${TELEOP_ARGS[@]+"${TELEOP_ARGS[@]}"}"
cleanup
