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
import sys
import json
import numpy as np
from urllib.parse import urlparse, parse_qs

PORT = 8000
WEB_DIR = Path(__file__).parent
PROJECT_ROOT = WEB_DIR.parent
MODELS_DIR = WEB_DIR / "models"

sys.path.insert(0, str(PROJECT_ROOT))
from web.scan_manager import manager as scan_manager  # noqa: E402
from src.data_processing import ChangeDetector  # noqa: E402

HTML_PAGES = ('/index.html', '/viewer.html', '/live.html', '/floorplan.html',
              '/scan.html', '/walkthrough.html', '/compare.html')


class ProjectHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    """Custom handler that serves web files, project documentation, and the
    JSON API for scan control / scan comparison."""

    def __init__(self, *args, **kwargs):
        # Serve from project root to access docs and markdown files
        super().__init__(*args, directory=str(PROJECT_ROOT), **kwargs)

    def translate_path(self, path):
        """Translate URL path to filesystem path, handling docs and examples."""
        # First get the parent's translation
        translated = super().translate_path(path)

        # Check if file exists - if not, try without URL encoding issues
        if not os.path.exists(translated):
            # Handle potential issues with path resolution on Windows
            clean_path = path.split('?')[0].split('#')[0]  # Remove query/fragment
            if clean_path.startswith('/'):
                clean_path = clean_path[1:]
            potential_path = PROJECT_ROOT / clean_path.replace('/', os.sep)
            if potential_path.exists():
                return str(potential_path)

        return translated

    def do_GET(self):
        parsed = urlparse(self.path)

        if parsed.path.startswith('/api/'):
            return self._handle_api(parsed.path, parse_qs(parsed.query))

        # Redirect root to web/index.html
        if self.path == '/' or self.path == '':
            self.path = '/web/index.html'
        # Redirect paths without web/ prefix for web assets
        elif self.path.startswith(HTML_PAGES):
            self.path = '/web' + self.path
        elif self.path.startswith('/js/') or self.path.startswith('/css/') or self.path.startswith('/models/'):
            self.path = '/web' + self.path
        return super().do_GET()

    # ------------------------------------------------------------------
    # JSON API
    # ------------------------------------------------------------------
    def _send_json(self, payload: dict, status: int = 200) -> None:
        body = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_api(self, path: str, query: dict) -> None:
        try:
            if path == '/api/scans':
                return self._send_json(self._list_scans())
            if path == '/api/compare':
                return self._send_json(self._compare_scans(query))
            if path == '/api/scan/start':
                name = (query.get('name') or [None])[0]
                if not name:
                    return self._send_json({"ok": False, "error": "Missing 'name' parameter"}, 400)
                duration = query.get('duration')
                duration = float(duration[0]) if duration else None
                return self._send_json(scan_manager.start(name, duration))
            if path == '/api/scan/stop':
                return self._send_json(scan_manager.stop())
            if path == '/api/scan/status':
                return self._send_json(scan_manager.status())

            return self._send_json({"error": f"Unknown API endpoint: {path}"}, 404)
        except Exception as e:
            return self._send_json({"error": str(e)}, 500)

    def _list_scans(self) -> dict:
        """List saved scans from web/models/index.json, falling back to a
        directory scan if the index is missing or out of date."""
        entries = []
        index_file = MODELS_DIR / 'index.json'
        indexed_files = set()

        if index_file.exists():
            try:
                entries = json.loads(index_file.read_text()).get('scans', [])
                indexed_files = {e['file'] for e in entries}
            except (json.JSONDecodeError, OSError):
                entries = []

        if MODELS_DIR.exists():
            for json_file in sorted(MODELS_DIR.glob('*.json')):
                if json_file.name == 'index.json' or json_file.name in indexed_files:
                    continue
                entries.append({"name": json_file.stem, "file": json_file.name})

        for entry in entries:
            file_path = MODELS_DIR / entry['file']
            entry['size_bytes'] = file_path.stat().st_size if file_path.exists() else 0

        return {"scans": entries}

    def _load_scan_points(self, filename: str) -> np.ndarray:
        # Basename only -- never let a query parameter escape MODELS_DIR.
        safe_name = Path(filename).name
        file_path = MODELS_DIR / safe_name
        if not file_path.exists():
            raise FileNotFoundError(f"Scan not found: {safe_name}")

        data = json.loads(file_path.read_text())
        vertices = np.array(data['vertices'], dtype=np.float64).reshape(-1, 3)
        return vertices

    def _compare_scans(self, query: dict) -> dict:
        baseline_name = (query.get('baseline') or [None])[0]
        current_name = (query.get('current') or [None])[0]
        if not baseline_name or not current_name:
            return {"error": "Both 'baseline' and 'current' query parameters are required"}

        voxel_size = float((query.get('voxel') or [0.05])[0])
        max_points = int((query.get('max_points') or [3000])[0])

        baseline_points = self._load_scan_points(baseline_name)
        current_points = self._load_scan_points(current_name)

        detector = ChangeDetector(voxel_size=voxel_size)
        report = detector.detect(baseline_points, current_points, align=True)

        def cap(points: np.ndarray) -> list:
            if len(points) > max_points:
                idx = np.random.choice(len(points), max_points, replace=False)
                points = points[idx]
            return points.tolist()

        return {
            "fitness": report.fitness,
            "added_count": len(report.added_points),
            "removed_count": len(report.removed_points),
            "persistent_count": len(report.persistent_points),
            "added_points": cap(report.added_points),
            "removed_points": cap(report.removed_points),
            "clusters": [
                {
                    "change_type": c.change_type,
                    "centroid": c.centroid.tolist(),
                    "num_points": c.num_points,
                }
                for c in report.clusters
            ],
        }

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
        print(f"  Live:     http://localhost:{PORT}/live.html")
        print()
        print("For live streaming, also run: python web/stream_server.py")
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