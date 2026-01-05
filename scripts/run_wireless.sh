#!/bin/bash

# Get project root directory (parent of scripts/)
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")
cd "$PROJECT_ROOT"

# Configuration
PORT_Bridge=9090
PORT_HTTPS=8000

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
if [[ ! -f "certs/cert.pem" || ! -f "certs/key.pem" ]]; then
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
python web/https_server.py $PORT_HTTPS > /dev/null 2>&1 &
HTTPS_PID=$!

# Wait a bit
sleep 1

# Start ROS Bridge in foreground
echo "🌉 Starting ROS Bridge on port $PORT_Bridge..."
python src/webxr_ros_bridge.py --cert certs/cert.pem --key certs/key.pem --port $PORT_Bridge
