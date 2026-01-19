"""
Simple HTTP server for the 3D Trail Viewer web application.
Run this to view the 3D visualizations in your browser.

Serves from project root to allow documentation access.
"""

import http.server
import socketserver
import webbrowser
from pathlib import Path
import os

PORT = 8000
WEB_DIR = Path(__file__).parent
PROJECT_ROOT = WEB_DIR.parent

class ProjectHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    """Custom handler that serves web files and project documentation."""

    def __init__(self, *args, **kwargs):
        # Serve from project root to access docs and markdown files
        super().__init__(*args, directory=str(PROJECT_ROOT), **kwargs)

    def do_GET(self):
        # Redirect root to web/index.html
        if self.path == '/' or self.path == '':
            self.path = '/web/index.html'
        # Redirect paths without web/ prefix for web assets
        elif self.path.startswith('/index.html') or self.path.startswith('/viewer.html'):
            self.path = '/web' + self.path
        elif self.path.startswith('/js/') or self.path.startswith('/css/') or self.path.startswith('/models/'):
            self.path = '/web' + self.path
        return super().do_GET()

    def end_headers(self):
        # Add CORS headers and markdown content type
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()

    def guess_type(self, path):
        # Serve .md files as text
        if path.endswith('.md'):
            return 'text/plain; charset=utf-8'
        return super().guess_type(path)


def main():
    # Change to project root
    os.chdir(PROJECT_ROOT)

    with socketserver.TCPServer(("", PORT), ProjectHTTPRequestHandler) as httpd:
        print("=" * 60)
        print("  LiDAR Trail Mapper - Web Server")
        print("=" * 60)
        print(f"Server running at: http://localhost:{PORT}")
        print(f"Project root: {PROJECT_ROOT}")
        print()
        print("Pages:")
        print(f"  Home:     http://localhost:{PORT}/")
        print(f"  Viewer:   http://localhost:{PORT}/viewer.html")
        print()
        print("Press Ctrl+C to stop the server")
        print("=" * 60)

        # Try to open browser automatically
        try:
            webbrowser.open(f'http://localhost:{PORT}/')
        except:
            pass

        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n\nServer stopped.")


if __name__ == "__main__":
    main()