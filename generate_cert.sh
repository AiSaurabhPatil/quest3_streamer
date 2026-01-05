#!/bin/bash

# Generate a self-signed certificate for local development
echo "Generating self-signed certificate (cert.pem) and key (key.pem)..."
echo "Common Name (CN) will be set to 'localhost' but works for IP access with warning."

openssl req -new -x509 -newkey rsa:2048 -nodes -sha256 \
    -subj "/C=US/ST=Dev/L=Local/O=Dev/CN=localhost" \
    -keyout key.pem -out cert.pem -days 365

echo ""
echo "✅ Certificate generated:"
echo "   - cert.pem"
echo "   - key.pem"
echo ""
echo "Usage:"
echo "   HTTPS Server: python https_server.py"
echo "   ROS Bridge:   python webxr_ros_bridge.py --cert cert.pem --key key.pem"
