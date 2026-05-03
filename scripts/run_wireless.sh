#!/bin/bash

# Get project root directory (parent of scripts/)
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")
cd "$PROJECT_ROOT"

CONFIG_FILE="$PROJECT_ROOT/config/config.yaml"
PYTHON_BIN="${PYTHON_BIN:-python3}"
if [[ -x "$PROJECT_ROOT/.venv/bin/python" ]]; then
    PYTHON_BIN="$PROJECT_ROOT/.venv/bin/python"
fi

config_value() {
    "$PYTHON_BIN" "$PROJECT_ROOT/src/config_loader.py" get --config "$CONFIG_FILE" "$1"
}

PORT_Bridge=$(config_value server.websocket_port)
PORT_HTTPS=$(config_value server.https_port)
CERT_PATH=$(config_value paths.certs.cert)
KEY_PATH=$(config_value paths.certs.key)

# Cleanup function to kill background processes on exit
cleanup() {
    echo "Stopping servers..."
    kill $HTTPS_PID
    exit
}
trap cleanup SIGINT

# Activate virtual environment
source .venv/bin/activate

# Check for certificates
if [[ ! -f "$CERT_PATH" || ! -f "$KEY_PATH" ]]; then
    echo "⚠️  Certificates not found. Generating them now..."
    bash scripts/generate_cert.sh
fi

echo "🚀 Starting Wireless WebXR Streamer"
echo "-----------------------------------"

# Get local IP
IP=$(hostname -I | awk '{print $1}')
echo "👉 Quest URL:   https://$IP:$PORT_HTTPS/web/webxr_streamer.html"
echo "👉 PC Server IP: $IP"
echo "👉 Port:         $PORT_Bridge"
echo "-----------------------------------"

# Start HTTPS Server in background
echo "📦 Starting HTTPS Server on port $PORT_HTTPS..."
"$PYTHON_BIN" web/https_server.py "$PORT_HTTPS" --cert "$CERT_PATH" --key "$KEY_PATH" > /dev/null 2>&1 &
HTTPS_PID=$!

# Wait a bit
sleep 1

# Start ROS Bridge in foreground
echo "🌉 Starting ROS Bridge on port $PORT_Bridge..."
"$PYTHON_BIN" -m src.launch.webxr_bridge --config "$CONFIG_FILE" --cert "$CERT_PATH" --key "$KEY_PATH" --port "$PORT_Bridge"
