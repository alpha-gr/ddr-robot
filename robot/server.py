#!/usr/bin/env python3
# serve_controller.py
# Serve il file controller.html sulla rete locale (porta 8000)
# Accessibile da PC o telefono: http://<IP_PC>:8000/controller.html

import http.server
import socketserver

PORT = 8000

Handler = http.server.SimpleHTTPRequestHandler

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"Server HTTP avviato sulla porta {PORT}")
    print("Apri sul telefono: http://<IP_PC>:{PORT}/controller.html")
    # stampa ip del pc
    print("IP del PC:", http.server.socket.gethostbyname(http.server.socket.gethostname()))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nServer fermato manualmente")
        httpd.server_close()
