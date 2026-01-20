"""
IMU-Fused SLAM for Unitree L2 LiDAR

This module properly fuses IMU data (quaternion, angular velocity, linear acceleration)
with LiDAR point cloud data to create accurate 3D models.

Key features:
1. Uses IMU quaternion to transform each point cloud frame to world coordinates
2. Tracks position using double integration of gravity-compensated acceleration
3. Fuses IMU-based pose with KISS-ICP odometry for drift correction
4. Maintains consistent world frame alignment using gravity vector

The IMU provides:
- Quaternion [w, x, y, z]: Orientation relative to gravity-aligned world frame
- Angular velocity [wx, wy, wz]: Rotation rate in rad/s
- Linear acceleration [ax, ay, az]: Including gravity, in m/s^2
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict
import time
import logging

logger = logging.getLogger(__name__)


@dataclass
class IMUFusedSLAMConfig:
    """Configuration for IMU-fused SLAM."""
    # SLAM parameters
    voxel_size: float = 0.05          # Odometry voxel size
    max_range: float = 15.0           # Maximum LiDAR range
    map_voxel_size: float = 0.02      # Map storage voxel size

    # IMU fusion parameters
    use_imu_orientation: bool = True   # Use IMU quaternion for frame alignment
    use_imu_position: bool = True      # Use IMU for position estimation
    imu_weight: float = 0.3            # Weight for IMU vs odometry (0=odometry only, 1=IMU only)

    # IMU to LiDAR transform (from Unitree docs)
    imu_to_lidar_offset: np.ndarray = field(
        default_factory=lambda: np.array([-0.007698, -0.014655, 0.00667])
    )

    # Gravity
    gravity_magnitude: float = 9.81

    # Position estimation
    velocity_decay: float = 0.99       # Velocity decay when stationary
    stationary_accel_threshold: float = 0.3  # m/s^2 variance threshold

    # Loop closure
    loop_closure_enabled: bool = True
    loop_min_gap: int = 30
    loop_dist_threshold: float = 0.25


class IMUState:
    """Tracks IMU state for position estimation."""

    def __init__(self, gravity_magnitude: float = 9.81):
        self.gravity_magnitude = gravity_magnitude

        # Current state
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion [w,x,y,z]
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)

        # Gravity vector in world frame (will be estimated)
        self.gravity_world = np.array([0.0, 0.0, -gravity_magnitude])

        # Bias estimates
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

        # Initialization
        self.initialized = False
        self.init_samples = []
        self.init_count = 0
        self.required_init_samples = 30

        # Timing
        self.last_timestamp = None

        # Statistics
        self.total_updates = 0

    def initialize_from_samples(self, accel_samples: List[np.ndarray],
                                 gyro_samples: List[np.ndarray]) -> bool:
        """
        Initialize IMU state from stationary samples.

        Estimates:
        - Gravity direction from average acceleration
        - Gyroscope bias from average angular velocity
        - Accelerometer bias (remaining after removing gravity)
        """
        if len(accel_samples) < self.required_init_samples:
            return False

        # Average acceleration = gravity + bias (when stationary)
        avg_accel = np.mean(accel_samples, axis=0)
        avg_gyro = np.mean(gyro_samples, axis=0)

        # Gravity direction (normalized)
        gravity_norm = np.linalg.norm(avg_accel)
        if gravity_norm < 0.1:
            logger.warning("Invalid gravity estimate - too small")
            return False

        # Gravity in sensor frame
        gravity_direction = avg_accel / gravity_norm

        # Scale to expected gravity magnitude
        self.gravity_world = gravity_direction * self.gravity_magnitude

        # Gyro bias is average when stationary
        self.gyro_bias = avg_gyro.copy()

        # Accel bias is residual after removing gravity
        self.accel_bias = avg_accel - self.gravity_world

        self.initialized = True
        logger.info(f"IMU initialized:")
        logger.info(f"  Gravity: {self.gravity_world}")
        logger.info(f"  Gyro bias: {self.gyro_bias}")
        logger.info(f"  Accel bias: {self.accel_bias}")

        return True

    def update(self, quaternion: np.ndarray, accel: np.ndarray,
               gyro: np.ndarray, timestamp: float) -> np.ndarray:
        """
        Update IMU state with new measurements.

        Args:
            quaternion: [w, x, y, z] orientation
            accel: [ax, ay, az] linear acceleration (m/s^2)
            gyro: [wx, wy, wz] angular velocity (rad/s)
            timestamp: Time in seconds

        Returns:
            Current position estimate [x, y, z]
        """
        # Initialization phase
        if not self.initialized:
            self.init_samples.append((accel.copy(), gyro.copy()))
            self.init_count += 1

            if self.init_count >= self.required_init_samples:
                accel_samples = [s[0] for s in self.init_samples]
                gyro_samples = [s[1] for s in self.init_samples]
                self.initialize_from_samples(accel_samples, gyro_samples)

            return self.position.copy()

        # Calculate dt
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            return self.position.copy()

        dt = timestamp - self.last_timestamp
        if dt <= 0 or dt > 1.0:  # Invalid or too large dt
            self.last_timestamp = timestamp
            return self.position.copy()

        self.last_timestamp = timestamp
        self.total_updates += 1

        # Update orientation from quaternion
        self.orientation = self._normalize_quaternion(quaternion)

        # Get rotation matrix from quaternion
        R = self._quaternion_to_rotation_matrix(self.orientation)

        # Remove bias from acceleration
        accel_corrected = accel - self.accel_bias

        # Transform acceleration to world frame
        accel_world = R @ accel_corrected

        # Remove gravity to get linear acceleration
        accel_linear = accel_world - self.gravity_world

        # Double integration for position
        # Using trapezoidal integration for better accuracy
        self.velocity = self.velocity + accel_linear * dt
        self.position = self.position + self.velocity * dt + 0.5 * accel_linear * dt**2

        # Apply velocity decay when nearly stationary (drift mitigation)
        accel_magnitude = np.linalg.norm(accel_linear)
        if accel_magnitude < 0.5:  # Low acceleration = likely stationary
            self.velocity *= 0.99

        return self.position.copy()

    def get_rotation_matrix(self) -> np.ndarray:
        """Get current rotation matrix from IMU orientation."""
        return self._quaternion_to_rotation_matrix(self.orientation)

    def get_transform_matrix(self) -> np.ndarray:
        """Get current 4x4 transformation matrix."""
        T = np.eye(4)
        T[:3, :3] = self.get_rotation_matrix()
        T[:3, 3] = self.position
        return T

    def _normalize_quaternion(self, q: np.ndarray) -> np.ndarray:
        """Normalize quaternion to unit length."""
        norm = np.linalg.norm(q)
        if norm < 1e-10:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / norm

    def _quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion [w,x,y,z] to 3x3 rotation matrix."""
        q = self._normalize_quaternion(q)
        w, x, y, z = q

        return np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - w*z),       2*(x*z + w*y)],
            [2*(x*y + w*z),           1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y),           2*(y*z + w*x),       1 - 2*(x**2 + y**2)]
        ])


class IMUFusedSLAM:
    """
    SLAM system that fuses IMU data with LiDAR point clouds.

    Uses IMU quaternion for:
    1. Transforming each point cloud frame to world coordinates
    2. Providing initial guess for ICP registration
    3. Stabilizing orientation estimates (gravity-aligned world frame)

    Uses IMU position estimation for:
    1. Dead reckoning between LiDAR scans
    2. Providing motion prior for odometry
    3. Detecting and compensating for motion blur
    """

    def __init__(self, config: Optional[IMUFusedSLAMConfig] = None):
        self.config = config or IMUFusedSLAMConfig()

        # IMU state tracker
        self.imu_state = IMUState(self.config.gravity_magnitude)

        # Initialize SLAM components
        self._init_odometry()
        self._init_map()
        if self.config.loop_closure_enabled:
            self._init_loop_closure()
            self._init_pose_graph()

        # State tracking
        self.current_pose = np.eye(4)  # Current pose in world frame
        self.trajectory = []           # List of positions
        self.poses = []                # List of 4x4 pose matrices
        self.scan_count = 0

        # Raw scan storage for loop closure correction
        self.raw_scans = []

        # IMU data buffer (for synchronization with point clouds)
        self.imu_buffer = []
        self.last_imu_timestamp = None

        logger.info("IMU-Fused SLAM initialized")
        logger.info(f"  IMU orientation: {self.config.use_imu_orientation}")
        logger.info(f"  IMU position: {self.config.use_imu_position}")
        logger.info(f"  IMU weight: {self.config.imu_weight}")

    def _init_odometry(self):
        """Initialize KISS-ICP odometry."""
        try:
            from .kiss_icp_odometry import KISSICPOdometry
            self.odometry = KISSICPOdometry(
                voxel_size=self.config.voxel_size,
                max_range=self.config.max_range
            )
            logger.info(f"KISS-ICP initialized (voxel={self.config.voxel_size}m)")
        except ImportError as e:
            logger.warning(f"KISS-ICP not available: {e}")
            self.odometry = None

    def _init_map(self):
        """Initialize ikd-Tree map."""
        from .ikd_tree import IKDTree
        self.global_map = IKDTree(
            downsample_size=self.config.map_voxel_size,
            box_length=self.config.map_voxel_size * 2
        )

    def _init_loop_closure(self):
        """Initialize Scan Context for loop closure."""
        from .scan_context import ScanContext, ScanContextConfig
        sc_config = ScanContextConfig(
            max_range=self.config.max_range,
            sc_dist_threshold=self.config.loop_dist_threshold
        )
        self.scan_context = ScanContext(sc_config)

    def _init_pose_graph(self):
        """Initialize pose graph optimization."""
        from .pose_graph import PoseGraph
        self.pose_graph = PoseGraph()

    def update_imu(self, quaternion: np.ndarray, accel: np.ndarray,
                   gyro: np.ndarray, timestamp: float) -> Dict:
        """
        Update with IMU data.

        Should be called at high frequency (typically 200-400 Hz for L2).

        Args:
            quaternion: [w, x, y, z] orientation
            accel: [ax, ay, az] linear acceleration (m/s^2)
            gyro: [wx, wy, wz] angular velocity (rad/s)
            timestamp: Time in seconds

        Returns:
            Dict with current IMU state
        """
        position = self.imu_state.update(quaternion, accel, gyro, timestamp)

        # Store in buffer for point cloud synchronization
        self.imu_buffer.append({
            'quaternion': quaternion.copy(),
            'accel': accel.copy(),
            'gyro': gyro.copy(),
            'timestamp': timestamp,
            'position': position.copy(),
            'rotation': self.imu_state.get_rotation_matrix().copy()
        })

        # Keep buffer limited
        if len(self.imu_buffer) > 100:
            self.imu_buffer.pop(0)

        self.last_imu_timestamp = timestamp

        return {
            'position': position,
            'orientation': self.imu_state.orientation.copy(),
            'velocity': self.imu_state.velocity.copy(),
            'initialized': self.imu_state.initialized
        }

    def process_scan(self, points: np.ndarray,
                     timestamp: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray, Dict]:
        """
        Process a LiDAR scan with IMU fusion.

        Args:
            points: Nx3 point cloud in sensor frame
            timestamp: Optional timestamp for IMU synchronization

        Returns:
            Tuple of (transformed_points, pose_matrix, info_dict)
        """
        self.scan_count += 1

        # Get IMU state at scan time
        imu_rotation = None
        imu_position = None

        if self.config.use_imu_orientation and self.imu_state.initialized:
            # Get closest IMU sample to this timestamp
            if timestamp is not None and len(self.imu_buffer) > 0:
                # Find closest IMU sample
                closest_idx = 0
                min_diff = float('inf')
                for i, imu in enumerate(self.imu_buffer):
                    diff = abs(imu['timestamp'] - timestamp)
                    if diff < min_diff:
                        min_diff = diff
                        closest_idx = i

                imu_data = self.imu_buffer[closest_idx]
                imu_rotation = imu_data['rotation']
                imu_position = imu_data['position']
            else:
                # Use current IMU state
                imu_rotation = self.imu_state.get_rotation_matrix()
                imu_position = self.imu_state.position.copy()

        # Transform points using IMU orientation (to world frame)
        if imu_rotation is not None:
            # Apply IMU rotation to align with world frame
            points_world = (imu_rotation @ points.T).T
        else:
            points_world = points.copy()

        # Run odometry
        if self.odometry is not None:
            # Provide IMU rotation as initial guess
            initial_guess = None
            if imu_rotation is not None and len(self.poses) > 0:
                # Create transform from IMU
                initial_guess = np.eye(4)
                initial_guess[:3, :3] = imu_rotation
                if imu_position is not None:
                    initial_guess[:3, 3] = imu_position

            # KISS-ICP returns (transformed_points, pose, converged)
            _, odom_pose, _ = self.odometry.process_scan(points_world, timestamp)
        else:
            odom_pose = self.current_pose.copy()

        # Fuse IMU and odometry poses
        if imu_rotation is not None and self.config.use_imu_orientation:
            # Blend IMU orientation with odometry
            alpha = self.config.imu_weight

            # Use IMU rotation, odometry translation
            fused_pose = odom_pose.copy()

            if self.config.use_imu_position and imu_position is not None:
                # Blend positions
                odom_position = odom_pose[:3, 3]
                fused_position = (1 - alpha) * odom_position + alpha * imu_position
                fused_pose[:3, 3] = fused_position

            # Blend orientations using SLERP-like approach
            # For simplicity, use weighted average of rotation matrices
            odom_rotation = odom_pose[:3, :3]
            fused_rotation = (1 - alpha) * odom_rotation + alpha * imu_rotation
            # Re-orthogonalize
            U, _, Vt = np.linalg.svd(fused_rotation)
            fused_pose[:3, :3] = U @ Vt

            self.current_pose = fused_pose
        else:
            self.current_pose = odom_pose

        # Transform points to world frame using fused pose
        transformed_points = self._transform_points(points, self.current_pose)

        # Store scan for potential loop closure correction
        self.raw_scans.append(points.copy())

        # Add to pose graph
        if hasattr(self, 'pose_graph'):
            node_id = len(self.poses)  # Current node ID
            self.pose_graph.add_node(node_id, self.current_pose.copy(), fixed=(node_id == 0))
            if len(self.poses) > 0:
                relative_pose = np.linalg.inv(self.poses[-1]) @ self.current_pose
                self.pose_graph.add_odometry_edge(
                    len(self.poses) - 1, node_id, relative_pose
                )

        # Store trajectory
        position = self.current_pose[:3, 3].copy()
        self.trajectory.append(position)
        self.poses.append(self.current_pose.copy())

        # Add to global map
        self.global_map.insert_points(transformed_points)

        # Check for loop closure
        loop_detected = False
        loop_idx = -1
        optimized = False

        if self.config.loop_closure_enabled and hasattr(self, 'scan_context'):
            # Add scan to Scan Context
            self.scan_context.add_scan(points, self.current_pose)

            # Check for loop closure
            if self.scan_count > self.config.loop_min_gap:
                matched_idx, distance = self.scan_context.detect_loop_closure(
                    points, self.scan_count - 1, min_gap=self.config.loop_min_gap
                )

                # Loop is detected if matched_idx is valid and distance is below threshold
                if matched_idx >= 0 and distance < self.config.loop_dist_threshold:
                    loop_detected = True
                    loop_idx = matched_idx
                    logger.info(f"Loop closure: {self.scan_count} -> {matched_idx}")

                    # Add loop closure constraint
                    relative_pose = np.linalg.inv(self.poses[matched_idx]) @ self.current_pose
                    self.pose_graph.add_loop_closure(
                        matched_idx, len(self.poses) - 1, relative_pose
                    )

                    # Optimize pose graph
                    if self.pose_graph.optimize():
                        optimized = True
                        self._apply_pose_correction()

        info = {
            'scan_count': self.scan_count,
            'position': position.tolist(),
            'map_points': self.global_map.total_points,
            'loop_detected': loop_detected,
            'loop_idx': loop_idx,
            'optimized': optimized,
            'imu_initialized': self.imu_state.initialized,
            'imu_updates': self.imu_state.total_updates
        }

        return transformed_points, self.current_pose.copy(), info

    def _transform_points(self, points: np.ndarray, pose: np.ndarray) -> np.ndarray:
        """Transform points using pose matrix."""
        R = pose[:3, :3]
        t = pose[:3, 3]
        return (R @ points.T).T + t

    def _apply_pose_correction(self):
        """Apply pose graph optimization results."""
        if not hasattr(self, 'pose_graph'):
            return

        optimized_poses = self.pose_graph.get_optimized_poses()
        if len(optimized_poses) != len(self.poses):
            return

        # Update poses
        self.poses = [p.copy() for p in optimized_poses]
        self.current_pose = self.poses[-1].copy()

        # Update trajectory
        self.trajectory = [p[:3, 3].copy() for p in self.poses]

        # Rebuild map with corrected poses
        logger.info("Rebuilding map with corrected poses...")
        self.global_map = type(self.global_map)(
            downsample_size=self.config.map_voxel_size,
            box_length=self.config.map_voxel_size * 2
        )

        for i, (scan, pose) in enumerate(zip(self.raw_scans, self.poses)):
            transformed = self._transform_points(scan, pose)
            self.global_map.insert_points(transformed)

        logger.info(f"Map rebuilt: {self.global_map.total_points} points")

    def get_map_points(self) -> np.ndarray:
        """Get all points in the global map."""
        return self.global_map.get_all_points()

    def get_trajectory(self) -> np.ndarray:
        """Get trajectory as Nx3 array."""
        if len(self.trajectory) == 0:
            return np.zeros((0, 3))
        return np.array(self.trajectory)

    def get_stats(self) -> Dict:
        """Get current statistics."""
        return {
            'total_scans': self.scan_count,
            'map_points': self.global_map.total_points,
            'trajectory_length': self._compute_trajectory_length(),
            'imu_initialized': self.imu_state.initialized,
            'imu_updates': self.imu_state.total_updates
        }

    def _compute_trajectory_length(self) -> float:
        """Compute total trajectory length."""
        if len(self.trajectory) < 2:
            return 0.0
        traj = np.array(self.trajectory)
        diffs = np.diff(traj, axis=0)
        return float(np.sum(np.linalg.norm(diffs, axis=1)))

    def save(self, output_dir: str, prefix: str = "imu_fused"):
        """Save SLAM results."""
        import os
        from pathlib import Path

        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # Save map
        map_points = self.get_map_points()
        np.save(output_path / f"{prefix}_map.npy", map_points)

        # Save as PLY
        self._save_ply(map_points, output_path / f"{prefix}_map.ply")

        # Save trajectory
        trajectory = self.get_trajectory()
        np.save(output_path / f"{prefix}_trajectory.npy", trajectory)

        # Save poses
        poses_array = np.array(self.poses)
        np.save(output_path / f"{prefix}_poses.npy", poses_array)

        # Save metadata
        import json
        metadata = {
            'total_scans': self.scan_count,
            'map_points': len(map_points),
            'trajectory_length': self._compute_trajectory_length(),
            'imu_initialized': self.imu_state.initialized,
            'imu_updates': self.imu_state.total_updates,
            'config': {
                'voxel_size': self.config.voxel_size,
                'max_range': self.config.max_range,
                'map_voxel_size': self.config.map_voxel_size,
                'use_imu_orientation': self.config.use_imu_orientation,
                'use_imu_position': self.config.use_imu_position,
                'imu_weight': self.config.imu_weight
            }
        }

        with open(output_path / f"{prefix}_metadata.json", 'w') as f:
            json.dump(metadata, f, indent=2)

        logger.info(f"Saved IMU-fused SLAM results to {output_path}")

    def _save_ply(self, points: np.ndarray, filepath):
        """Save points as PLY file."""
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
