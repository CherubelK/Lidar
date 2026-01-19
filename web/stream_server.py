"""
WebSocket server for live LiDAR streaming to browser.

Run this server, then open live.html in your browser to see
real-time point cloud visualization.

Usage:
    python web/stream_server.py

Then open: http://localhost:8000/live.html
"""

import asyncio
import json
import sys
from pathlib import Path
import numpy as np

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    import websockets
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False
    print("Installing websockets...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "websockets"])
    import websockets

from src.lidar_interface.unitree_l2_udp import UnitreeL2UDP, UnitreeL2Config
from src.data_processing.complete_slam import CompleteSLAM, SLAMConfig
from src.data_processing.imu_integration import IMUIntegration


class LiDARStreamServer:
    """WebSocket server that streams LiDAR data to connected clients."""

    def __init__(self):
        self.clients = set()
        self.lidar = None
        self.slam = None
        self.imu = None
        self.is_scanning = False
        self.scan_task = None

    async def register(self, websocket):
        """Register a new client connection."""
        self.clients.add(websocket)
        print(f"[+] Client connected ({len(self.clients)} total)")
        await self.send_status(websocket, "Connected to LiDAR stream server", "info")

    async def unregister(self, websocket):
        """Unregister a client connection."""
        self.clients.discard(websocket)
        print(f"[-] Client disconnected ({len(self.clients)} total)")

    async def send_status(self, websocket, message, level="info"):
        """Send a status message to a client."""
        await websocket.send(json.dumps({
            "type": "status",
            "message": message,
            "level": level
        }))

    async def broadcast(self, message):
        """Broadcast a message to all connected clients."""
        if self.clients:
            await asyncio.gather(
                *[client.send(message) for client in self.clients],
                return_exceptions=True
            )

    async def handle_message(self, websocket, message):
        """Handle incoming messages from clients."""
        try:
            data = json.loads(message)
            command = data.get("command")

            if command == "start":
                if not self.is_scanning:
                    await self.start_scan(websocket, data)
            elif command == "stop":
                await self.stop_scan(websocket)
            else:
                await self.send_status(websocket, f"Unknown command: {command}", "error")

        except json.JSONDecodeError:
            await self.send_status(websocket, "Invalid JSON message", "error")
        except Exception as e:
            await self.send_status(websocket, f"Error: {str(e)}", "error")

    async def start_scan(self, websocket, settings):
        """Start a LiDAR scan session."""
        try:
            duration = settings.get("duration", 60)
            voxel_size = settings.get("voxel_size", 0.05)
            loop_closure = settings.get("loop_closure", True)
            imu_deskew = settings.get("imu_deskew", True)

            await self.send_status(websocket, "Initializing LiDAR...", "info")

            # Initialize LiDAR
            config = UnitreeL2Config()
            self.lidar = UnitreeL2UDP(config)

            if not self.lidar.connect():
                await self.send_status(websocket, "Failed to connect to LiDAR", "error")
                return

            await self.send_status(websocket, "LiDAR connected", "success")

            # Initialize SLAM
            slam_config = SLAMConfig(
                voxel_size=voxel_size,
                max_range=15.0,
                map_voxel_size=voxel_size,
                local_map_radius=20.0,
                loop_closure_enabled=loop_closure,
                loop_min_gap=30,
                loop_dist_threshold=0.3,
                save_trajectory=True,
                save_map=True
            )
            self.slam = CompleteSLAM(slam_config)

            # Initialize IMU if deskewing is enabled
            if imu_deskew:
                self.imu = IMUIntegration(gravity=9.81)
                await self.send_status(websocket, "IMU integration enabled", "info")

            self.is_scanning = True
            await self.send_status(websocket, f"Starting scan ({duration}s)...", "info")

            # Start scanning in a background task
            self.scan_task = asyncio.create_task(
                self.scan_loop(websocket, duration, imu_deskew)
            )

        except Exception as e:
            await self.send_status(websocket, f"Error starting scan: {str(e)}", "error")
            self.is_scanning = False

    async def scan_loop(self, websocket, duration, use_imu):
        """Main scanning loop that streams data to clients."""
        import time
        start_time = time.time()
        scan_count = 0
        accumulated_points = []
        frames_per_scan = 2

        try:
            while self.is_scanning and (time.time() - start_time) < duration:
                # Get point cloud from LiDAR
                result = self.lidar.get_point_cloud_with_intensity()

                if result is None:
                    await asyncio.sleep(0.01)
                    continue

                # Handle IMU data
                if isinstance(result, dict):
                    if use_imu and self.imu:
                        self.imu.update(result, timestamp=time.time())
                    continue

                points, intensities = result

                if len(points) < 50:
                    continue

                # Deskew with IMU if available
                if use_imu and self.imu and self.imu.initialized:
                    points = self.imu.deskew_point_cloud_fast(points, scan_duration=0.1)

                accumulated_points.append(points)

                # Process accumulated frames
                if len(accumulated_points) >= frames_per_scan:
                    combined_points = np.vstack(accumulated_points)
                    accumulated_points = []

                    # Downsample if needed
                    if len(combined_points) > 10000:
                        indices = np.random.choice(len(combined_points), 10000, replace=False)
                        combined_points = combined_points[indices]

                    # Process with SLAM
                    transformed_points, pose, info = self.slam.process_scan(combined_points)
                    scan_count += 1

                    # Sample points for sending (limit bandwidth)
                    if len(transformed_points) > 2000:
                        indices = np.random.choice(len(transformed_points), 2000, replace=False)
                        send_points = transformed_points[indices]
                    else:
                        send_points = transformed_points

                    # Send to clients
                    message = json.dumps({
                        "type": "points",
                        "points": send_points.tolist(),
                        "scan_count": scan_count,
                        "loop_closure": info.get("loop_detected", False),
                        "position": pose[:3, 3].tolist()
                    })
                    await self.broadcast(message)

                # Small delay to prevent overwhelming
                await asyncio.sleep(0.02)

            # Scan complete
            await self.broadcast(json.dumps({
                "type": "complete",
                "total_scans": scan_count,
                "total_points": self.slam.get_stats()["map_points"]
            }))

        except asyncio.CancelledError:
            await self.send_status(websocket, "Scan cancelled", "warning")
        except Exception as e:
            await self.send_status(websocket, f"Scan error: {str(e)}", "error")
        finally:
            self.is_scanning = False
            if self.lidar:
                self.lidar.disconnect()

    async def stop_scan(self, websocket):
        """Stop the current scan."""
        self.is_scanning = False
        if self.scan_task:
            self.scan_task.cancel()
            try:
                await self.scan_task
            except asyncio.CancelledError:
                pass
        await self.send_status(websocket, "Scan stopped", "warning")

    async def handler(self, websocket, path):
        """Handle WebSocket connections."""
        await self.register(websocket)
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            await self.unregister(websocket)
            if self.is_scanning:
                await self.stop_scan(websocket)


async def main():
    server = LiDARStreamServer()

    print("=" * 60)
    print("  LiDAR Live Stream Server")
    print("=" * 60)
    print()
    print("WebSocket server running on: ws://localhost:8765")
    print()
    print("To view the live stream:")
    print("  1. Start the web server: python web/server.py")
    print("  2. Open: http://localhost:8000/live.html")
    print()
    print("Press Ctrl+C to stop")
    print("=" * 60)

    async with websockets.serve(server.handler, "localhost", 8765):
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nServer stopped.")
