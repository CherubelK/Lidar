"""
LiDAR Data Capture Module
Handles capturing and saving LiDAR data streams to disk.
"""

import numpy as np
import open3d as o3d
import logging
from pathlib import Path
from datetime import datetime
from typing import Optional, List
import json

from .unitree_l2 import UnitreeL2LiDAR, LiDARConfig

logger = logging.getLogger(__name__)


class LiDARDataCapture:
    """
    High-level interface for capturing and saving LiDAR data sessions.
    """

    def __init__(self,
                 output_dir: str = "data/raw",
                 lidar_config: Optional[LiDARConfig] = None):
        """
        Initialize the data capture system.

        Args:
            output_dir: Directory to save captured data
            lidar_config: Configuration for the LiDAR sensor
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.lidar = UnitreeL2LiDAR(lidar_config)
        self.session_dir: Optional[Path] = None
        self.frame_count = 0

    def start_session(self, session_name: Optional[str] = None) -> bool:
        """
        Start a new data capture session.

        Args:
            session_name: Name for this session. If None, uses timestamp.

        Returns:
            bool: True if session started successfully
        """
        if session_name is None:
            session_name = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.session_dir = self.output_dir / session_name
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.frame_count = 0

        # Save session metadata
        metadata = {
            "session_name": session_name,
            "start_time": datetime.now().isoformat(),
            "config": {
                "ip_address": self.lidar.config.ip_address,
                "port": self.lidar.config.port,
                "frame_rate": self.lidar.config.frame_rate,
                "range_min": self.lidar.config.range_min,
                "range_max": self.lidar.config.range_max
            }
        }

        metadata_file = self.session_dir / "metadata.json"
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)

        # Connect to sensor and start streaming
        if not self.lidar.connect():
            logger.error("Failed to connect to LiDAR sensor")
            return False

        if not self.lidar.start_streaming():
            logger.error("Failed to start LiDAR streaming")
            return False

        logger.info(f"Started capture session: {session_name}")
        return True

    def capture_frame(self, save: bool = True) -> Optional[np.ndarray]:
        """
        Capture a single frame of point cloud data.

        Args:
            save: Whether to save the frame to disk

        Returns:
            np.ndarray: Captured point cloud or None if failed
        """
        if self.session_dir is None:
            logger.error("No active session. Call start_session() first.")
            return None

        point_cloud = self.lidar.get_point_cloud()

        if point_cloud is None:
            logger.warning("Failed to capture frame")
            return None

        if save:
            self._save_point_cloud(point_cloud, self.frame_count)

        self.frame_count += 1
        return point_cloud

    def capture_continuous(self,
                          duration_seconds: Optional[float] = None,
                          num_frames: Optional[int] = None) -> int:
        """
        Capture point cloud data continuously.

        Args:
            duration_seconds: How long to capture (in seconds). If None, uses num_frames.
            num_frames: Number of frames to capture. If None, uses duration_seconds.

        Returns:
            int: Number of frames captured
        """
        if self.session_dir is None:
            logger.error("No active session. Call start_session() first.")
            return 0

        frames_captured = 0
        start_time = datetime.now()

        logger.info(f"Starting continuous capture (duration: {duration_seconds}s, frames: {num_frames})")

        try:
            while True:
                # Check stopping conditions
                if num_frames is not None and frames_captured >= num_frames:
                    break

                if duration_seconds is not None:
                    elapsed = (datetime.now() - start_time).total_seconds()
                    if elapsed >= duration_seconds:
                        break

                # Capture frame
                point_cloud = self.capture_frame(save=True)

                if point_cloud is not None:
                    frames_captured += 1

                    if frames_captured % 10 == 0:
                        logger.info(f"Captured {frames_captured} frames")

        except KeyboardInterrupt:
            logger.info("Capture interrupted by user")

        logger.info(f"Continuous capture complete. Total frames: {frames_captured}")
        return frames_captured

    def end_session(self) -> None:
        """End the current capture session."""
        if self.session_dir is None:
            return

        # Update metadata with end time and frame count
        metadata_file = self.session_dir / "metadata.json"
        if metadata_file.exists():
            with open(metadata_file, 'r') as f:
                metadata = json.load(f)

            metadata["end_time"] = datetime.now().isoformat()
            metadata["total_frames"] = self.frame_count

            with open(metadata_file, 'w') as f:
                json.dump(metadata, f, indent=2)

        self.lidar.stop_streaming()
        self.lidar.disconnect()

        logger.info(f"Session ended. Captured {self.frame_count} frames to {self.session_dir}")
        self.session_dir = None

    def _save_point_cloud(self, points: np.ndarray, frame_number: int) -> None:
        """
        Save point cloud to disk in multiple formats.

        Args:
            points: Nx3 array of point coordinates
            frame_number: Frame number for filename
        """
        filename_base = f"frame_{frame_number:06d}"

        # Save as NumPy array (fast, efficient)
        npy_file = self.session_dir / f"{filename_base}.npy"
        np.save(npy_file, points)

        # Save as PCD file (Open3D format, good for visualization)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd_file = self.session_dir / f"{filename_base}.pcd"
        o3d.io.write_point_cloud(str(pcd_file), pcd)

    def load_session_frames(self, session_name: str) -> List[np.ndarray]:
        """
        Load all frames from a previous session.

        Args:
            session_name: Name of the session to load

        Returns:
            List of point cloud arrays
        """
        session_path = self.output_dir / session_name

        if not session_path.exists():
            logger.error(f"Session not found: {session_name}")
            return []

        frames = []
        npy_files = sorted(session_path.glob("frame_*.npy"))

        logger.info(f"Loading {len(npy_files)} frames from {session_name}")

        for npy_file in npy_files:
            points = np.load(npy_file)
            frames.append(points)

        return frames

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.end_session()