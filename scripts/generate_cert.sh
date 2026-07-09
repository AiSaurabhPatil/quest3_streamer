#!/bin/bash
# =============================================================================
# generate_cert.sh — Generate a self-signed SSL certificate
# =============================================================================
# WebXR requires HTTPS. This generates a self-signed cert for local use.
#
# Usage:
#   ./scripts/generate_cert.sh              # localhost only
#   ./scripts/generate_cert.sh --ip <LAN_IP>  # include workstation LAN IP as SAN
#
# The Quest browser (and Chrome-based browsers) require the server's IP/hostname
# to appear in the certificate's Subject Alternative Names (SAN). Without a SAN
# the browser will show ERR_CERT_COMMON_NAME_INVALID.
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Parse optional --ip argument
# ---------------------------------------------------------------------------
EXTRA_IP=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ip)
            EXTRA_IP="$2"
            shift 2
            ;;
        --ip=*)
            EXTRA_IP="${1#*=}"
            shift
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

# If --ip was not passed, auto-detect LAN IP
if [[ -z "$EXTRA_IP" ]]; then
    EXTRA_IP=$(hostname -I | awk '{print $1}')
fi

# ---------------------------------------------------------------------------
# Resolve paths
# ---------------------------------------------------------------------------
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")
cd "$PROJECT_ROOT"

mkdir -p certs

# ---------------------------------------------------------------------------
# Build the Subject Alternative Names (SAN) extension config
# Includes localhost, 127.0.0.1, and the detected/provided LAN IP.
# ---------------------------------------------------------------------------
SAN_CONFIG=$(mktemp /tmp/san_cert_XXXXXX.cnf)
trap 'rm -f "$SAN_CONFIG"' EXIT

cat > "$SAN_CONFIG" <<EOF
[req]
distinguished_name = req_distinguished_name
x509_extensions    = v3_req
prompt             = no

[req_distinguished_name]
C  = US
ST = Dev
L  = Local
O  = Dev
CN = localhost

[v3_req]
subjectAltName = @alt_names

[alt_names]
DNS.1 = localhost
IP.1  = 127.0.0.1
IP.2  = $EXTRA_IP
EOF

echo "Generating self-signed SSL certificate..."
echo "  Subject Alternative Names: localhost, 127.0.0.1, $EXTRA_IP"
echo "  Output: certs/cert.pem, certs/key.pem"
echo ""

openssl req -new -x509 -newkey rsa:2048 -nodes -sha256 \
    -config "$SAN_CONFIG" \
    -extensions v3_req \
    -keyout certs/key.pem \
    -out  certs/cert.pem \
    -days 365

echo ""
echo "Certificate generated successfully:"
echo "   certs/cert.pem"
echo "   certs/key.pem"
echo ""
echo "On Quest browser:"
echo "  1. Navigate to: https://$EXTRA_IP:8000/web/webxr_streamer.html"
echo "  2. Tap 'Advanced' -> 'Proceed to $EXTRA_IP (unsafe)' to accept the cert"
echo "  3. You only need to do this once per certificate"
echo ""
echo "Usage:"
echo "   HTTPS Server : python web/https_server.py 8000 --cert certs/cert.pem --key certs/key.pem"
echo "   WebXR Bridge : python -m src.launch.webxr_bridge --cert certs/cert.pem --key certs/key.pem"
