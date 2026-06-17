"""
Scan Session Process Manager
Starts, stops, and polls the status of a scripts/scan_session.py subprocess
on behalf of the web app's start/stop scan controls.

Only one scan session can run at a time -- starting a second one while one
is active returns an error rather than launching a competing capture
process against the same sensor.
"""
import json
import signal
import subprocess
import sys
from pathlib import Path
from typing import Optional

PROJECT_ROOT = Path(__file__).parent.parent
SCAN_SCRIPT = PROJECT_ROOT / "scripts" / "scan_session.py"
STATUS_FILE = PROJECT_ROOT / "web" / ".scan_status.json"


class ScanSessionManager:
    def __init__(self):
        self._process: Optional[subprocess.Popen] = None
        self._session_name: Optional[str] = None

    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def start(self, session_name: str, duration: Optional[float] = None) -> dict:
        if self.is_running():
            return {"ok": False, "error": f"A scan ('{self._session_name}') is already running. Stop it first."}

        cmd = [sys.executable, str(SCAN_SCRIPT), "--session-name", session_name,
               "--status-file", str(STATUS_FILE)]
        if duration is not None:
            cmd += ["--duration", str(duration)]

        popen_kwargs = {}
        if sys.platform == "win32":
            # Needed so CTRL_BREAK_EVENT (sent by stop()) targets only this
            # process, not the whole console/process tree.
            popen_kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP

        try:
            self._process = subprocess.Popen(cmd, cwd=str(PROJECT_ROOT), **popen_kwargs)
        except OSError as e:
            return {"ok": False, "error": f"Failed to launch scan process: {e}"}

        self._session_name = session_name
        return {"ok": True, "session_name": session_name}

    def stop(self) -> dict:
        if not self.is_running():
            return {"ok": False, "error": "No scan is currently running."}

        if sys.platform == "win32":
            self._process.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            self._process.send_signal(signal.SIGTERM)

        try:
            self._process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self._process.kill()
            return {"ok": True, "warning": "Scan process did not exit gracefully and was force-killed."}

        return {"ok": True, "session_name": self._session_name}

    def status(self) -> dict:
        """
        Read the live status file written by the running (or just-finished)
        scan_session.py process. Falls back to process-liveness if the
        status file doesn't exist yet (e.g. right after start()).
        """
        if STATUS_FILE.exists():
            try:
                data = json.loads(STATUS_FILE.read_text())
                # Reconcile with actual process state in case it died
                # without writing a final status (e.g. killed externally).
                if data.get("running") and not self.is_running():
                    data["running"] = False
                    data["phase"] = data.get("phase", "unknown")
                    data["error"] = data.get("error", "Scan process ended unexpectedly.")
                return data
            except (json.JSONDecodeError, OSError):
                pass

        return {"running": self.is_running(), "session_name": self._session_name, "phase": "starting"}


# Shared singleton -- the HTTP server is single-process, so one manager
# instance per server process is sufficient to track the one active scan.
manager = ScanSessionManager()
