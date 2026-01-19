"""
Complete SLAM System with Loop Closure

Integrates:
- KISS-ICP for odometry (frame-to-frame matching)
- ikd-Tree for efficient map storage
- Scan Context for loop closure detection
- Pose Graph Optimization for drift correction

Based on Point-LIO architecture but implemented in pure Python
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from pathlib import Path
import json
import time
import logging

# Try to import Open3D, fall back to scipy-based ICP if not available
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

from .kiss_icp_odometry import KISSICPOdometry
from .ikd_tree import IKDTree
from .scan_context import ScanContext, ScanContextConfig
from .pose_graph import PoseGraph
from .icp_registration import ICPRegistration

logger = logging.getLogger(__name__)


@dataclass
class SLAMConfig:
    """Configuration for complete SLAM system"""
    # Odometry
    voxel_size: float = 0.05          # Voxel size for KISS-ICP
    max_range: float = 20.0           # Maximum point range

    # Map
    map_voxel_size: float = 0.1       # Voxel size for ikd-Tree map
    local_map_radius: float = 50.0    # Radius of local map around robot

    # Loop closure
    loop_closure_enabled: bool = True
    loop_min_gap: int = 50            # Minimum frames between loop candidates
    loop_dist_threshold: float = 0.2  # Scan Context distance threshold

    # Optimization
    optimize_every_n_loops: int = 1   # Run optimization after N loop closures

    # Output
    save_trajectory: bool = True
    save_map: bool = True


class CompleteSLAM:
    """
    Complete SLAM system with loop closure and pose graph optimization

    Pipeline:
    1. Process scan with KISS-ICP (odometry)
    2. Add points to ikd-Tree map
    3. Create Scan Context descriptor
    4. Check for loop closure
    5. If loop found: add constraint and optimize pose graph
    6. Correct map with optimized poses
    """

    def __init__(self, config: Optional[SLAMConfig] = None):
        """
        Initialize complete SLAM system

        Args:
            config: SLAM configuration
        """
        self.config = config or SLAMConfig()

        # Initialize components
        self._init_odometry()
        self._init_map()
        self._init_loop_closure()
        self._init_pose_graph()

        # State
        self.current_pose = np.eye(4)
        self.scan_count = 0
        self.loop_count = 0
        self.last_optimization_loop = 0

        # Storage
        self.all_poses: List[np.ndarray] = []
        self.all_scans: List[np.ndarray] = []
        self.timestamps: List[float] = []

        logger.info("Complete SLAM system initialized")
        logger.info(f"  Odometry: KISS-ICP (voxel={self.config.voxel_size}m)")
        logger.info(f"  Map: ikd-Tree (voxel={self.config.map_voxel_size}m)")
        logger.info(f"  Loop closure: {'Enabled' if self.config.loop_closure_enabled else 'Disabled'}")

    def _init_odometry(self) -> None:
        """Initialize KISS-ICP odometry"""
        self.odometry = KISSICPOdometry(
            voxel_size=self.config.voxel_size,
            max_range=self.config.max_range
        )

    def _init_map(self) -> None:
        """Initialize ikd-Tree map"""
        self.global_map = IKDTree(
            downsample_size=self.config.map_voxel_size,
            box_length=self.config.map_voxel_size * 2
        )

    def _init_loop_closure(self) -> None:
        """Initialize Scan Context for loop closure"""
        sc_config = ScanContextConfig(
            max_range=self.config.max_range,
            sc_dist_threshold=self.config.loop_dist_threshold
        )
        self.scan_context = ScanContext(sc_config)

    def _init_pose_graph(self) -> None:
        """Initialize pose graph"""
        self.pose_graph = PoseGraph()

    def process_scan(self, points: np.ndarray,
                    timestamp: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray, dict]:
        """
        Process a new LiDAR scan

        Args:
            points: Nx3 point cloud
            timestamp: Scan timestamp (optional)

        Returns:
            (transformed_points, current_pose, info_dict)
        """
        if timestamp is None:
            timestamp = time.time()

        info = {
            'scan_idx': self.scan_count,
            'loop_detected': False,
            'loop_idx': -1,
            'optimized': False
        }

        # 1. Process with KISS-ICP odometry
        transformed_points, pose, success = self.odometry.process_scan(points)

        if not success:
            logger.warning(f"Odometry failed for scan {self.scan_count}")
            return points, self.current_pose, info

        self.current_pose = pose
        self.all_poses.append(pose.copy())
        self.all_scans.append(points.copy())
        self.timestamps.append(timestamp)

        # 2. Add to pose graph
        self._add_to_pose_graph(pose)

        # 3. Add to global map (ikd-Tree)
        self._add_to_map(transformed_points)

        # 4. Add to Scan Context database
        self.scan_context.add_scan(points, timestamp)

        # 5. Check for loop closure
        if self.config.loop_closure_enabled and self.scan_count >= self.config.loop_min_gap:
            loop_idx, loop_dist = self._detect_loop_closure(points)

            if loop_idx >= 0:
                info['loop_detected'] = True
                info['loop_idx'] = loop_idx
                info['loop_distance'] = loop_dist

                self.loop_count += 1

                # 6. Optimize if needed
                if self.loop_count - self.last_optimization_loop >= self.config.optimize_every_n_loops:
                    self._optimize()
                    info['optimized'] = True
                    self.last_optimization_loop = self.loop_count

        self.scan_count += 1

        return transformed_points, self.current_pose, info

    def _add_to_pose_graph(self, pose: np.ndarray) -> None:
        """Add pose to pose graph with odometry constraint"""
        node_id = self.scan_count

        # Add node
        self.pose_graph.add_node(node_id, pose, fixed=(node_id == 0))

        # Add odometry edge to previous pose
        if node_id > 0:
            prev_pose = self.all_poses[-2] if len(self.all_poses) >= 2 else np.eye(4)
            relative_pose = np.linalg.inv(prev_pose) @ pose
            self.pose_graph.add_odometry_edge(node_id - 1, node_id, relative_pose)

    def _add_to_map(self, points: np.ndarray) -> None:
        """Add transformed points to global map"""
        # Add to ikd-Tree
        self.global_map.insert_points(points)

        # Remove distant points if map is getting large
        if self.global_map.size() > 1000000:  # 1M points
            center = self.current_pose[:3, 3]
            radius = self.config.local_map_radius

            # Delete points outside local radius
            # This is a simplification - real systems use more sophisticated methods
            box_min = center - radius * 2
            box_max = center + radius * 2

            # Only keep points in expanded box (don't delete recent points)
            # For simplicity, we'll skip this optimization for now

    def _detect_loop_closure(self, points: np.ndarray) -> Tuple[int, float]:
        """
        Detect loop closure using Scan Context

        Returns:
            (match_idx, distance) or (-1, inf) if no loop
        """
        match_idx, dist = self.scan_context.detect_loop_closure(
            points,
            current_idx=self.scan_count,
            min_gap=self.config.loop_min_gap
        )

        if match_idx >= 0:
            # Verify with ICP
            relative_pose = self._compute_loop_transform(
                self.all_scans[match_idx],
                points,
                match_idx
            )

            if relative_pose is not None:
                # Add loop closure constraint
                self.pose_graph.add_loop_closure(
                    match_idx,
                    self.scan_count,
                    relative_pose,
                    confidence=1.0 - dist  # Higher confidence for lower distance
                )

                logger.info(f"Loop closure added: {match_idx} -> {self.scan_count}")
                return match_idx, dist

        return -1, float('inf')

    def _compute_loop_transform(self, source_points: np.ndarray,
                                target_points: np.ndarray,
                                source_idx: int) -> Optional[np.ndarray]:
        """
        Compute relative transformation for loop closure using ICP

        Args:
            source_points: Points from earlier scan
            target_points: Points from current scan
            source_idx: Index of source scan

        Returns:
            4x4 relative transformation or None if failed
        """
        try:
            if HAS_OPEN3D:
                # Use Open3D for ICP (faster, more robust)
                pcd_source = o3d.geometry.PointCloud()
                pcd_source.points = o3d.utility.Vector3dVector(source_points)

                pcd_target = o3d.geometry.PointCloud()
                pcd_target.points = o3d.utility.Vector3dVector(target_points)

                # Downsample
                voxel_size = self.config.voxel_size * 2
                pcd_source = pcd_source.voxel_down_sample(voxel_size)
                pcd_target = pcd_target.voxel_down_sample(voxel_size)

                # Estimate normals
                pcd_source.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30)
                )
                pcd_target.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30)
                )

                # ICP registration
                threshold = voxel_size * 3
                reg = o3d.pipelines.registration.registration_icp(
                    pcd_source, pcd_target, threshold, np.eye(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
                )

                if reg.fitness > 0.3:
                    return reg.transformation
                else:
                    logger.warning(f"ICP registration failed (fitness={reg.fitness:.3f})")
                    return None
            else:
                # Use our scipy-based ICP implementation
                icp = ICPRegistration(max_iterations=50, tolerance=1e-6)
                transform, fitness = icp.register(source_points, target_points)

                if fitness > 0.3:  # Fitness threshold (30% overlap)
                    return transform
                else:
                    logger.warning(f"ICP registration failed (fitness={fitness:.3f})")
                    return None

        except Exception as e:
            logger.error(f"Loop closure ICP failed: {e}")
            return None

    def _optimize(self) -> None:
        """Run pose graph optimization and correct map"""
        logger.info("Running pose graph optimization...")

        # Optimize poses
        final_error = self.pose_graph.optimize()

        # Get corrected poses
        corrected_poses = self.pose_graph.get_optimized_poses()

        # Update stored poses
        for i, pose in enumerate(corrected_poses):
            if i < len(self.all_poses):
                self.all_poses[i] = pose

        # Update current pose
        if len(corrected_poses) > 0:
            self.current_pose = corrected_poses[-1]

        # Rebuild map with corrected poses
        self._rebuild_map_with_corrected_poses(corrected_poses)

        logger.info(f"Optimization complete. Final error: {final_error:.6f}")

    def _rebuild_map_with_corrected_poses(self, poses: List[np.ndarray]) -> None:
        """Rebuild global map using corrected poses"""
        logger.info("Rebuilding map with corrected poses...")

        # Clear current map
        self.global_map.clear()

        # Re-add all scans with corrected poses
        for i, (scan, pose) in enumerate(zip(self.all_scans, poses)):
            # Transform points with corrected pose
            points_h = np.hstack([scan, np.ones((len(scan), 1))])
            transformed = (pose @ points_h.T).T[:, :3]

            # Add to map
            self.global_map.insert_points(transformed)

        logger.info(f"Map rebuilt: {self.global_map.size()} points")

    def get_map_points(self) -> np.ndarray:
        """Get all points in the global map"""
        return self.global_map.get_all_points()

    def get_trajectory(self) -> np.ndarray:
        """Get robot trajectory as Nx3 positions"""
        if len(self.all_poses) == 0:
            return np.array([]).reshape(0, 3)

        return np.array([pose[:3, 3] for pose in self.all_poses])

    def get_stats(self) -> dict:
        """Get SLAM statistics"""
        return {
            'total_scans': self.scan_count,
            'total_poses': len(self.all_poses),
            'map_points': self.global_map.size(),
            'loop_closures': self.loop_count,
            'scan_contexts': self.scan_context.get_num_scans(),
            'pose_graph': self.pose_graph.get_stats(),
            'trajectory_length': self._compute_trajectory_length()
        }

    def _compute_trajectory_length(self) -> float:
        """Compute total trajectory length"""
        if len(self.all_poses) < 2:
            return 0.0

        trajectory = self.get_trajectory()
        distances = np.linalg.norm(np.diff(trajectory, axis=0), axis=1)
        return float(np.sum(distances))

    def save(self, output_dir: str, prefix: str = "slam") -> None:
        """
        Save SLAM results to disk

        Args:
            output_dir: Output directory
            prefix: Filename prefix
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # Save map
        if self.config.save_map:
            map_points = self.get_map_points()
            if len(map_points) > 0:
                np.save(output_path / f"{prefix}_map.npy", map_points)

                # Also save as PLY if Open3D available
                if HAS_OPEN3D:
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(map_points)
                    o3d.io.write_point_cloud(str(output_path / f"{prefix}_map.ply"), pcd)
                else:
                    # Save simple PLY without Open3D
                    self._save_ply_simple(map_points, output_path / f"{prefix}_map.ply")

                logger.info(f"Saved map: {len(map_points)} points")

        # Save trajectory
        if self.config.save_trajectory:
            trajectory = self.get_trajectory()
            if len(trajectory) > 0:
                np.save(output_path / f"{prefix}_trajectory.npy", trajectory)
                logger.info(f"Saved trajectory: {len(trajectory)} poses")

        # Save poses
        if len(self.all_poses) > 0:
            poses_array = np.array(self.all_poses)
            np.save(output_path / f"{prefix}_poses.npy", poses_array)

        # Save Scan Contexts
        self.scan_context.save(str(output_path / f"{prefix}_scan_contexts.npz"))

        # Save metadata
        metadata = {
            'config': {
                'voxel_size': self.config.voxel_size,
                'max_range': self.config.max_range,
                'map_voxel_size': self.config.map_voxel_size,
                'loop_closure_enabled': self.config.loop_closure_enabled,
                'loop_min_gap': self.config.loop_min_gap,
                'loop_dist_threshold': self.config.loop_dist_threshold
            },
            'stats': self.get_stats(),
            'timestamps': self.timestamps
        }

        with open(output_path / f"{prefix}_metadata.json", 'w') as f:
            json.dump(metadata, f, indent=2, default=str)

        logger.info(f"SLAM results saved to {output_path}")

    def _save_ply_simple(self, points: np.ndarray, filepath: Path) -> None:
        """Save point cloud as PLY file without Open3D"""
        with open(filepath, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            for p in points:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

    def reset(self) -> None:
        """Reset SLAM system"""
        self._init_odometry()
        self._init_map()
        self._init_loop_closure()
        self._init_pose_graph()

        self.current_pose = np.eye(4)
        self.scan_count = 0
        self.loop_count = 0
        self.last_optimization_loop = 0

        self.all_poses = []
        self.all_scans = []
        self.timestamps = []

        logger.info("SLAM system reset")


if __name__ == "__main__":
    # Demo
    logging.basicConfig(level=logging.INFO)
    print("=== Complete SLAM Demo ===\n")

    # Create SLAM system
    config = SLAMConfig(
        voxel_size=0.05,
        max_range=20.0,
        loop_closure_enabled=True,
        loop_min_gap=20  # Lower for demo
    )

    slam = CompleteSLAM(config)

    # Generate synthetic scans (simulating a loop)
    print("Generating synthetic scans...")

    def make_scan(x, y, yaw):
        """Generate a simple room-like scan"""
        points = []

        # Simulate walls at different distances
        for angle in np.linspace(-np.pi/2, np.pi/2, 100):
            # Front wall
            dist = 5.0 / np.cos(angle) if abs(angle) < np.pi/4 else 10.0
            px = dist * np.cos(angle + yaw) + x
            py = dist * np.sin(angle + yaw) + y
            pz = np.random.uniform(0, 2.5)
            points.append([px, py, pz])

        points = np.array(points)
        points += np.random.randn(*points.shape) * 0.05  # Add noise

        return points

    # Simulate robot moving in a square loop
    trajectory = []
    num_scans = 80

    for i in range(num_scans):
        # Move in square: forward, right, back, left
        t = i / num_scans * 4  # 0-4

        if t < 1:  # Forward
            x, y, yaw = t * 10, 0, 0
        elif t < 2:  # Right
            x, y, yaw = 10, (t-1) * 10, np.pi/2
        elif t < 3:  # Back
            x, y, yaw = 10 - (t-2) * 10, 10, np.pi
        else:  # Left (back to start)
            x, y, yaw = 0, 10 - (t-3) * 10, -np.pi/2

        trajectory.append([x, y])

        # Generate scan
        scan = make_scan(x, y, yaw)

        # Process with SLAM
        _, pose, info = slam.process_scan(scan)

        if info['loop_detected']:
            print(f"Scan {i}: Loop detected! Matched with scan {info['loop_idx']}")

        if i % 20 == 0:
            print(f"Scan {i}: position = ({pose[0,3]:.2f}, {pose[1,3]:.2f})")

    # Get results
    print("\n=== Results ===")
    stats = slam.get_stats()
    print(f"Total scans: {stats['total_scans']}")
    print(f"Loop closures: {stats['loop_closures']}")
    print(f"Map points: {stats['map_points']}")
    print(f"Trajectory length: {stats['trajectory_length']:.2f}m")

    # Check loop closure quality
    final_trajectory = slam.get_trajectory()
    start = final_trajectory[0]
    end = final_trajectory[-1]
    gap = np.linalg.norm(end - start)
    print(f"\nLoop closure gap: {gap:.3f}m")
    print(f"  Start: ({start[0]:.2f}, {start[1]:.2f})")
    print(f"  End: ({end[0]:.2f}, {end[1]:.2f})")

    # Save results
    slam.save("data/slam_demo", prefix="demo")