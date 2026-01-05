#!/usr/bin/env python3
import http.server
import ssl
import sys

def run_server(port=8000):
    server_address = ('0.0.0.0', port)
    httpd = http.server.HTTPServer(server_address, http.server.SimpleHTTPRequestHandler)

    # Wrap the socket with SSL
    try:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(certfile="cert.pem", keyfile="key.pem")
        httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
    except FileNotFoundError:
        print("❌ Error: cert.pem or key.pem not found.")
        print("   Run './generate_cert.sh' first.")
        sys.exit(1)

    print(f"🔒 HTTPS Server running at https://0.0.0.0:{port}/")
    print("   (Accept the security warning in your browser)")
    httpd.serve_forever()

if __name__ == "__main__":
    port = 8000
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    run_server(port)
