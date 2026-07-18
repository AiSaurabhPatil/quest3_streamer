#!/usr/bin/env python3
import http.server
import ssl
import sys
import os
import argparse

def run_server(port=8000, cert_file="certs/cert.pem", key_file="certs/key.pem"):
    # Get project root (parent of web/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    # Change to project root so web files are served correctly
    os.chdir(project_root)
    
    server_address = ('0.0.0.0', port)
    httpd = http.server.HTTPServer(server_address, http.server.SimpleHTTPRequestHandler)

    # Wrap the socket with SSL
    try:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(certfile=cert_file, keyfile=key_file)
        httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
    except FileNotFoundError:
        print(f"❌ Error: {cert_file} or {key_file} not found.")
        print("   Run './scripts/generate_cert.sh' first.")
        sys.exit(1)

    print(f"🔒 HTTPS Server running at https://0.0.0.0:{port}/")
    print(f"   Serving from: {project_root}")
    print("   (Accept the security warning in your browser)")
    httpd.serve_forever()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="HTTPS static server for Quest WebXR")
    parser.add_argument("port", nargs="?", type=int, default=8000)
    parser.add_argument("--cert", default="certs/cert.pem", help="Path to SSL certificate")
    parser.add_argument("--key", default="certs/key.pem", help="Path to SSL key")
    args = parser.parse_args()
    run_server(args.port, args.cert, args.key)
