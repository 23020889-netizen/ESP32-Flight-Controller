"""
Launcher script for UAV HITL Testbed Monitor 3D Dashboard
Runs a local lightweight HTTP server and opens your browser.
Usage:
    python tools/run_monitor_web.py
"""

import sys
import os
import webbrowser
from http.server import HTTPServer, SimpleHTTPRequestHandler

PORT = 8000
DIRECTORY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hitl_monitor_web")


class CustomHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def log_message(self, format, *args):
        # Keep terminal clean, only log errors
        pass


def main():
    print("=" * 65)
    print("   UAV HITL TESTBED MONITOR - 3D REAL-TIME DASHBOARD")
    print(f"   Dang khoi chay may chu Web cuc bo tai: http://localhost:{PORT}")
    print("=" * 65)

    os.chdir(DIRECTORY)
    server = HTTPServer(("127.0.0.1", PORT), CustomHandler)
    url = f"http://localhost:{PORT}/UAV%20HITL%20Testbed%20Monitor.html"
    print(f"\n[INFO] Dang mo trinh duyet Web tai:\n       {url}\n")
    print("[INFO] Bam 'Ket Noi ESP32 (WiFi)' tren Web de nhan Telemetry truc tiep!")
    print("[INFO] Nhan Ctrl + C trong Terminal de dung may chu.\n")
    
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] May chu Web da dung.")
        server.server_close()


if __name__ == '__main__':
    main()
