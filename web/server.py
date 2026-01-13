"""
Simple HTTP server for the 3D Trail Viewer web application.
Run this to view the 3D visualizations in your browser.
"""

import http.server
import socketserver
import webbrowser
from pathlib import Path

PORT = 8000
DIRECTORY = Path(__file__).parent

class MyHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(DIRECTORY), **kwargs)

    def end_headers(self):
        # Add CORS headers to allow local file access
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()

def main():
    with socketserver.TCPServer(("", PORT), MyHTTPRequestHandler) as httpd:
        print("=" * 60)
        print("3D Trail Viewer Server")
        print("=" * 60)
        print(f"Server running at: http://localhost:{PORT}")
        print(f"Serving directory: {DIRECTORY}")
        print()
        print("Open your browser and navigate to:")
        print(f"  http://localhost:{PORT}/index.html")
        print()
        print("Press Ctrl+C to stop the server")
        print("=" * 60)

        # Try to open browser automatically
        try:
            webbrowser.open(f'http://localhost:{PORT}/index.html')
        except:
            pass

        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n\nServer stopped.")

if __name__ == "__main__":
    main()