"""
Scan Session Runner
Standalone process invoked by the web app's "Start Scan" / "Stop Scan"
controls (see web/scan_manager.py). Connects to the real Unitree L2 sensor,
captures continuously, and writes a live status JSON file the web UI polls
for progress. Stops on SIGINT/SIGTERM (or CTRL_BREAK_EVENT on Windows, which
is how web/scan_manager.py asks a running session to stop gracefully) and on
exit saves the SLAM map/poses and exports a web-viewer-ready JSON so the new
scan immediately shows up in the scan library.

Run directly for testing without the web app:
    python scripts/scan_session.py --session-name test_scan --duration 30
"""
import argparse
import json
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.lidar_interface.unitree_l2_udp import UnitreeL2UDP
from src.data_processing.mesh_generator import MeshGenerator

DEFAULT_STATUS_FILE = PROJECT_ROOT / "web" / ".scan_status.json"
MODELS_DIR = PROJECT_ROOT / "web" / "models"
INDEX_FILE = MODELS_DIR / "index.json"

_stop_requested = False


def _request_stop(signum, frame):
    global _stop_requested
    _stop_requested = True


def _install_signal_handlers():
    """Register handlers so the process can be asked to stop gracefully.

    On Windows, a process group must receive CTRL_BREAK_EVENT (SIGBREAK)
    rather than SIGTERM, since SIGTERM isn't deliverable to another process
    on that platform. web/scan_manager.py launches this script with
    CREATE_NEW_PROCESS_GROUP and sends CTRL_BREAK_EVENT to stop it.
    """
    signal.signal(signal.SIGINT, _request_stop)
    if sys.platform == "win32":
        signal.signal(signal.SIGBREAK, _request_stop)
    else:
        signal.signal(signal.SIGTERM, _request_stop)


def write_status(status_file: Path, **fields) -> None:
    status_file.parent.mkdir(parents=True, exist_ok=True)
    payload = {"updated_at": datetime.now().isoformat(), **fields}
    # Write atomically-ish: build the new content fully before touching disk
    # so a status poll never reads a half-written file.
    tmp_file = status_file.with_suffix(".tmp")
    with open(tmp_file, "w") as f:
        json.dump(payload, f)
    tmp_file.replace(status_file)


def update_scan_index(session_name: str, json_filename: str) -> None:
    """Add (or update) this session's entry in web/models/index.json."""
    MODELS_DIR.mkdir(parents=True, exist_ok=True)

    entries = []
    if INDEX_FILE.exists():
        try:
            entries = json.loads(INDEX_FILE.read_text()).get("scans", [])
        except (json.JSONDecodeError, OSError):
            entries = []

    entries = [e for e in entries if e.get("name") != session_name]
    entries.append({"name": session_name, "file": json_filename})

    INDEX_FILE.write_text(json.dumps({"scans": entries}, indent=2))


def run_session(session_name: str, duration: float, status_file: Path) -> int:
    _install_signal_handlers()

    write_status(status_file, running=True, session_name=session_name,
                 phase="connecting", elapsed_seconds=0, points_captured=0)

    receiver = UnitreeL2UDP()
    if not receiver.connect():
        write_status(status_file, running=False, session_name=session_name,
                     phase="error", error="Could not connect to Unitree L2 sensor. "
                     "Check power, network config, and that nothing else is bound to the UDP port.")
        return 1

    write_status(status_file, running=True, session_name=session_name,
                 phase="scanning", elapsed_seconds=0, points_captured=0)

    all_points = []
    packet_count = 0
    start_time = time.time()
    last_status_write = start_time

    try:
        while not _stop_requested:
            elapsed = time.time() - start_time
            if duration is not None and elapsed >= duration:
                break

            result = receiver.get_point_cloud_with_intensity()
            if result is not None:
                points, _ = result
                all_points.append(points)
                packet_count += 1

            if time.time() - last_status_write >= 1.0:
                write_status(
                    status_file, running=True, session_name=session_name,
                    phase="scanning", elapsed_seconds=round(time.time() - start_time, 1),
                    points_captured=int(sum(len(p) for p in all_points)),
                    packet_count=packet_count,
                )
                last_status_write = time.time()
    finally:
        receiver.disconnect()

    if len(all_points) == 0:
        write_status(status_file, running=False, session_name=session_name,
                     phase="error", error="No points captured during the session.")
        return 1

    write_status(status_file, running=True, session_name=session_name,
                 phase="processing", elapsed_seconds=round(time.time() - start_time, 1),
                 points_captured=int(sum(len(p) for p in all_points)))

    merged_points = np.vstack(all_points)
    total_points = len(merged_points)

    downsample_factor = max(1, total_points // 50000)
    if downsample_factor > 1:
        merged_points = merged_points[::downsample_factor]

    mesh_gen = MeshGenerator()
    mesh_data = mesh_gen.create_trail_mesh(merged_points, method="delaunay")

    output_path = MODELS_DIR / f"{session_name}.json"
    mesh_gen.export_to_json(mesh_data, str(output_path))
    update_scan_index(session_name, output_path.name)

    write_status(
        status_file, running=False, session_name=session_name, phase="completed",
        elapsed_seconds=round(time.time() - start_time, 1),
        points_captured=total_points,
        mesh_vertices=mesh_data["num_vertices"],
        mesh_faces=mesh_data["num_faces"],
        output_file=output_path.name,
    )
    return 0


def main():
    parser = argparse.ArgumentParser(description="Run a Unitree L2 scan session for the web app")
    parser.add_argument("--session-name", default=None,
                         help="Name for this scan (default: timestamp)")
    parser.add_argument("--duration", type=float, default=None,
                         help="Max scan duration in seconds (default: run until stopped)")
    parser.add_argument("--status-file", default=str(DEFAULT_STATUS_FILE),
                         help="Path to write live status JSON")
    args = parser.parse_args()

    session_name = args.session_name or datetime.now().strftime("scan_%Y%m%d_%H%M%S")
    sys.exit(run_session(session_name, args.duration, Path(args.status_file)))


if __name__ == "__main__":
    main()
