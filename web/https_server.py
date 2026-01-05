#!/usr/bin/env python3
import http.server
import ssl
import sys
import os

def run_server(port=8000):
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
        context.load_cert_chain(certfile="certs/cert.pem", keyfile="certs/key.pem")
        httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
    except FileNotFoundError:
        print("❌ Error: certs/cert.pem or certs/key.pem not found.")
        print("   Run './scripts/generate_cert.sh' first.")
        sys.exit(1)

    print(f"🔒 HTTPS Server running at https://0.0.0.0:{port}/")
    print(f"   Serving from: {project_root}")
    print("   (Accept the security warning in your browser)")
    httpd.serve_forever()

if __name__ == "__main__":
    port = 8000
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    run_server(port)
