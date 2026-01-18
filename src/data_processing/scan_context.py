"""
Scan Context for LiDAR place recognition and loop closure detection

Based on: "Scan Context: Egocentric Spatial Descriptor for Place Recognition
          within 3D Point Cloud Map"
https://irap.kaist.ac.kr/publications/gkim-2018-iros.pdf

Scan Context encodes a point cloud into a 2D descriptor that is:
- Rotation invariant (via ring key matching)
- Efficient to compute and compare
- Robust for loop closure detection
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class ScanContextConfig:
    """Configuration for Scan Context descriptor"""
    # Descriptor dimensions
    num_sectors: int = 60      # Number of angular sectors (columns)
    num_rings: int = 20        # Number of radial rings (rows)

    # Range limits
    max_range: float = 80.0    # Maximum range (meters)
    min_range: float = 0.5     # Minimum range (meters)

    # Loop closure thresholds
    sc_dist_threshold: float = 0.2   # Scan context distance threshold
    num_candidates: int = 10          # Number of loop candidates to consider

    # Ring key for fast search
    tree_making_period: int = 50      # Build search tree every N scans


class ScanContext:
    """
    Scan Context descriptor for loop closure detection

    Creates a 2D descriptor from 3D point cloud by:
    1. Project points to BEV (bird's eye view)
    2. Divide into polar grid (rings × sectors)
    3. Encode max height in each cell
    """

    def __init__(self, config: Optional[ScanContextConfig] = None):
        """
        Initialize Scan Context

        Args:
            config: Configuration parameters
        """
        self.config = config or ScanContextConfig()

        # Storage for scan contexts and ring keys
        self.scan_contexts: List[np.ndarray] = []
        self.ring_keys: List[np.ndarray] = []
        self.timestamps: List[float] = []

        # Precompute sector and ring boundaries
        self._init_grid()

        logger.info(f"Scan Context initialized: {self.config.num_rings} rings × "
                   f"{self.config.num_sectors} sectors")

    def _init_grid(self) -> None:
        """Initialize polar grid parameters"""
        # Ring boundaries (uniform in range)
        self.ring_boundaries = np.linspace(
            self.config.min_range,
            self.config.max_range,
            self.config.num_rings + 1
        )

        # Sector boundaries (uniform in angle)
        self.sector_boundaries = np.linspace(
            -np.pi, np.pi,
            self.config.num_sectors + 1
        )

    def make_scan_context(self, points: np.ndarray) -> np.ndarray:
        """
        Create Scan Context descriptor from point cloud

        Args:
            points: Nx3 array of 3D points

        Returns:
            2D descriptor (num_rings × num_sectors)
        """
        # Initialize descriptor with -inf (no points)
        descriptor = np.full(
            (self.config.num_rings, self.config.num_sectors),
            -np.inf
        )

        # Convert to cylindrical coordinates
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # Range and angle
        ranges = np.sqrt(x**2 + y**2)
        angles = np.arctan2(y, x)

        # Filter points by range
        valid = (ranges >= self.config.min_range) & (ranges <= self.config.max_range)
        ranges = ranges[valid]
        angles = angles[valid]
        heights = z[valid]

        if len(ranges) == 0:
            return np.zeros((self.config.num_rings, self.config.num_sectors))

        # Compute ring and sector indices
        ring_indices = np.searchsorted(self.ring_boundaries[1:], ranges)
        ring_indices = np.clip(ring_indices, 0, self.config.num_rings - 1)

        sector_indices = np.searchsorted(self.sector_boundaries[1:], angles)
        sector_indices = np.clip(sector_indices, 0, self.config.num_sectors - 1)

        # Fill descriptor with max height in each cell
        for i in range(len(ranges)):
            r_idx = ring_indices[i]
            s_idx = sector_indices[i]
            if heights[i] > descriptor[r_idx, s_idx]:
                descriptor[r_idx, s_idx] = heights[i]

        # Replace -inf with 0 (no points in cell)
        descriptor[descriptor == -np.inf] = 0.0

        return descriptor

    def make_ring_key(self, scan_context: np.ndarray) -> np.ndarray:
        """
        Create ring key from Scan Context for fast search

        Ring key is rotation-invariant summary (mean of each ring)

        Args:
            scan_context: 2D Scan Context descriptor

        Returns:
            1D ring key (num_rings,)
        """
        # Mean of each ring (row)
        return np.mean(scan_context, axis=1)

    def add_scan(self, points: np.ndarray, timestamp: float = 0.0) -> int:
        """
        Add a new scan to the database

        Args:
            points: Nx3 point cloud
            timestamp: Scan timestamp (optional)

        Returns:
            Index of the added scan
        """
        # Create Scan Context
        sc = self.make_scan_context(points)
        self.scan_contexts.append(sc)

        # Create ring key
        rk = self.make_ring_key(sc)
        self.ring_keys.append(rk)

        self.timestamps.append(timestamp)

        return len(self.scan_contexts) - 1

    def detect_loop_closure(self, points: np.ndarray,
                           current_idx: int,
                           min_gap: int = 50) -> Tuple[int, float]:
        """
        Detect loop closure for current scan

        Args:
            points: Current Nx3 point cloud
            current_idx: Current scan index
            min_gap: Minimum index gap to consider (avoid recent scans)

        Returns:
            (best_match_idx, distance) or (-1, inf) if no loop found
        """
        if len(self.scan_contexts) < min_gap + 1:
            return -1, float('inf')

        # Create current Scan Context
        current_sc = self.make_scan_context(points)
        current_rk = self.make_ring_key(current_sc)

        # Find candidates using ring key (fast pre-filtering)
        candidates = self._find_candidates_by_ring_key(
            current_rk, current_idx, min_gap
        )

        if len(candidates) == 0:
            return -1, float('inf')

        # Compare with candidates using full Scan Context
        best_idx = -1
        best_dist = float('inf')
        best_yaw = 0

        for candidate_idx in candidates:
            dist, yaw = self._distance_with_yaw(
                current_sc,
                self.scan_contexts[candidate_idx]
            )

            if dist < best_dist:
                best_dist = dist
                best_idx = candidate_idx
                best_yaw = yaw

        # Check threshold
        if best_dist < self.config.sc_dist_threshold:
            logger.info(f"Loop closure detected: {current_idx} -> {best_idx}, "
                       f"dist={best_dist:.3f}, yaw={np.degrees(best_yaw):.1f}°")
            return best_idx, best_dist
        else:
            return -1, best_dist

    def _find_candidates_by_ring_key(self, query_rk: np.ndarray,
                                     current_idx: int,
                                     min_gap: int) -> List[int]:
        """
        Find loop closure candidates using ring key similarity

        Args:
            query_rk: Query ring key
            current_idx: Current scan index
            min_gap: Minimum index gap

        Returns:
            List of candidate indices
        """
        candidates = []
        distances = []

        max_idx = current_idx - min_gap

        for i in range(max_idx):
            # L2 distance between ring keys
            dist = np.linalg.norm(query_rk - self.ring_keys[i])
            candidates.append(i)
            distances.append(dist)

        if len(candidates) == 0:
            return []

        # Sort by distance and return top candidates
        sorted_indices = np.argsort(distances)
        num_candidates = min(self.config.num_candidates, len(candidates))

        return [candidates[i] for i in sorted_indices[:num_candidates]]

    def _distance_with_yaw(self, sc1: np.ndarray,
                          sc2: np.ndarray) -> Tuple[float, float]:
        """
        Compute distance between two Scan Contexts with yaw alignment

        Tries all column shifts to find best alignment (rotation invariance)

        Args:
            sc1: First Scan Context
            sc2: Second Scan Context

        Returns:
            (minimum_distance, best_yaw_offset)
        """
        num_sectors = self.config.num_sectors

        min_dist = float('inf')
        best_shift = 0

        # Try all column shifts
        for shift in range(num_sectors):
            # Shift sc2 by 'shift' columns
            sc2_shifted = np.roll(sc2, shift, axis=1)

            # Compute cosine distance (1 - cosine similarity)
            # Flatten and compute
            v1 = sc1.flatten()
            v2 = sc2_shifted.flatten()

            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)

            if norm1 < 1e-6 or norm2 < 1e-6:
                continue

            cos_sim = np.dot(v1, v2) / (norm1 * norm2)
            dist = 1.0 - cos_sim

            if dist < min_dist:
                min_dist = dist
                best_shift = shift

        # Convert shift to yaw angle
        yaw = (best_shift / num_sectors) * 2 * np.pi
        if yaw > np.pi:
            yaw -= 2 * np.pi

        return min_dist, yaw

    def get_relative_pose(self, idx1: int, idx2: int,
                         points1: np.ndarray,
                         points2: np.ndarray) -> np.ndarray:
        """
        Estimate relative pose between two scans using ICP refinement

        Args:
            idx1: First scan index
            idx2: Second scan index
            points1: First point cloud
            points2: Second point cloud

        Returns:
            4x4 transformation matrix from scan1 to scan2
        """
        import open3d as o3d

        # Get initial yaw estimate from Scan Context
        sc1 = self.scan_contexts[idx1]
        sc2 = self.scan_contexts[idx2]
        _, yaw = self._distance_with_yaw(sc1, sc2)

        # Create initial transform (rotation around Z)
        initial_transform = np.eye(4)
        initial_transform[0, 0] = np.cos(yaw)
        initial_transform[0, 1] = -np.sin(yaw)
        initial_transform[1, 0] = np.sin(yaw)
        initial_transform[1, 1] = np.cos(yaw)

        # Create Open3D point clouds
        pcd1 = o3d.geometry.PointCloud()
        pcd1.points = o3d.utility.Vector3dVector(points1)

        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(points2)

        # ICP refinement
        threshold = 0.5
        reg = o3d.pipelines.registration.registration_icp(
            pcd1, pcd2, threshold, initial_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
        )

        return reg.transformation

    def save(self, filepath: str) -> None:
        """Save Scan Context database to file"""
        np.savez(
            filepath,
            scan_contexts=np.array(self.scan_contexts),
            ring_keys=np.array(self.ring_keys),
            timestamps=np.array(self.timestamps),
            config_num_sectors=self.config.num_sectors,
            config_num_rings=self.config.num_rings,
            config_max_range=self.config.max_range,
            config_min_range=self.config.min_range,
            config_sc_dist_threshold=self.config.sc_dist_threshold
        )
        logger.info(f"Saved {len(self.scan_contexts)} Scan Contexts to {filepath}")

    def load(self, filepath: str) -> None:
        """Load Scan Context database from file"""
        data = np.load(filepath)

        self.scan_contexts = list(data['scan_contexts'])
        self.ring_keys = list(data['ring_keys'])
        self.timestamps = list(data['timestamps'])

        # Restore config
        self.config.num_sectors = int(data['config_num_sectors'])
        self.config.num_rings = int(data['config_num_rings'])
        self.config.max_range = float(data['config_max_range'])
        self.config.min_range = float(data['config_min_range'])
        self.config.sc_dist_threshold = float(data['config_sc_dist_threshold'])

        self._init_grid()

        logger.info(f"Loaded {len(self.scan_contexts)} Scan Contexts from {filepath}")

    def get_num_scans(self) -> int:
        """Get number of scans in database"""
        return len(self.scan_contexts)

    def visualize_scan_context(self, idx: int) -> None:
        """Visualize a Scan Context as heatmap (requires matplotlib)"""
        try:
            import matplotlib.pyplot as plt

            sc = self.scan_contexts[idx]

            plt.figure(figsize=(12, 4))
            plt.imshow(sc, aspect='auto', cmap='viridis')
            plt.colorbar(label='Max Height (m)')
            plt.xlabel('Sector (angle)')
            plt.ylabel('Ring (range)')
            plt.title(f'Scan Context #{idx}')
            plt.show()

        except ImportError:
            logger.warning("matplotlib not available for visualization")


if __name__ == "__main__":
    # Demo
    logging.basicConfig(level=logging.INFO)
    print("=== Scan Context Demo ===\n")

    # Create Scan Context manager
    sc_manager = ScanContext()

    # Generate synthetic point clouds
    np.random.seed(42)

    def make_room_scan(offset_x=0, offset_y=0, noise=0.1):
        """Generate a simple room-like point cloud"""
        points = []

        # Floor
        for x in np.linspace(-5, 5, 50):
            for y in np.linspace(-5, 5, 50):
                points.append([x + offset_x, y + offset_y, 0])

        # Walls
        for z in np.linspace(0, 3, 20):
            for x in np.linspace(-5, 5, 30):
                points.append([x + offset_x, -5 + offset_y, z])
                points.append([x + offset_x, 5 + offset_y, z])
            for y in np.linspace(-5, 5, 30):
                points.append([-5 + offset_x, y + offset_y, z])
                points.append([5 + offset_x, y + offset_y, z])

        points = np.array(points)
        points += np.random.randn(*points.shape) * noise

        return points

    # Add several scans
    print("Adding scans to database...")

    # Scans 0-49: moving forward
    for i in range(50):
        points = make_room_scan(offset_x=i * 0.5)
        sc_manager.add_scan(points, timestamp=i)

    # Scans 50-99: turning and coming back (creates loop)
    for i in range(50):
        offset = 25 - i * 0.5  # Moving back
        points = make_room_scan(offset_x=offset, offset_y=2)
        sc_manager.add_scan(points, timestamp=50 + i)

    print(f"Database has {sc_manager.get_num_scans()} scans")

    # Test loop closure detection
    print("\nTesting loop closure detection...")

    # Scan 90 should match somewhere around scan 10
    test_points = make_room_scan(offset_x=5, offset_y=2)
    match_idx, dist = sc_manager.detect_loop_closure(
        test_points,
        current_idx=90,
        min_gap=50
    )

    if match_idx >= 0:
        print(f"✓ Loop closure found: scan 90 matches scan {match_idx}")
        print(f"  Distance: {dist:.4f}")
    else:
        print(f"✗ No loop closure detected (best distance: {dist:.4f})")