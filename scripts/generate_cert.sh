#!/bin/bash

# Get project root directory (parent of scripts/)
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")
cd "$PROJECT_ROOT"

# Ensure certs directory exists
mkdir -p certs

# Generate a self-signed certificate for local development
echo "Generating self-signed certificate (certs/cert.pem) and key (certs/key.pem)..."
echo "Common Name (CN) will be set to 'localhost' but works for IP access with warning."

openssl req -new -x509 -newkey rsa:2048 -nodes -sha256 \
    -subj "/C=US/ST=Dev/L=Local/O=Dev/CN=localhost" \
    -keyout certs/key.pem -out certs/cert.pem -days 365

echo ""
echo "✅ Certificate generated:"
echo "   - certs/cert.pem"
echo "   - certs/key.pem"
echo ""
echo "Usage:"
echo "   HTTPS Server: python web/https_server.py"
echo "   ROS Bridge:   python src/webxr_ros_bridge.py --cert certs/cert.pem --key certs/key.pem"
