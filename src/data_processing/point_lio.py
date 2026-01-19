"""
Point-LIO: Point-based LiDAR-Inertial Odometry

Python implementation inspired by HKU-MARS Point-LIO (https://github.com/hku-mars/Point-LIO)

This implements an iterated Error-State Extended Kalman Filter (ES-EKF) for
tightly-coupled LiDAR-inertial odometry with:
- SO(3) manifold-based state representation
- Point-by-point measurement update
- Point-to-plane residuals
- IMU bias estimation
- Gravity vector estimation

References:
- Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry (2023)
- FAST-LIO2: Fast Direct LiDAR-Inertial Odometry (2022)
"""

import numpy as np
from scipy.spatial import cKDTree
from scipy.linalg import solve, qr
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import time


@dataclass
class PointLIOConfig:
    """Configuration for Point-LIO algorithm."""
    # IMU noise parameters
    gyr_cov: float = 0.01           # Gyroscope noise covariance
    acc_cov: float = 0.1            # Accelerometer noise covariance
    b_gyr_cov: float = 0.0001       # Gyro bias random walk
    b_acc_cov: float = 0.0001       # Accel bias random walk

    # LiDAR parameters
    lidar_meas_cov: float = 0.01    # Point measurement covariance
    plane_thr: float = 0.1          # Plane fitting threshold (m)
    match_s: float = 81.0           # Point-to-plane criterion
    num_match_points: int = 5       # Neighbors for plane fitting

    # Processing parameters
    voxel_size: float = 0.3         # Downsampling voxel size
    max_range: float = 50.0         # Maximum point range
    min_range: float = 0.5          # Minimum point range (blind zone)

    # Filter parameters
    max_iterations: int = 2         # iEKF iterations per update
    convergence_thr: float = 0.001  # Convergence threshold

    # IMU saturation limits
    satu_acc: float = 30.0          # Acceleration saturation (m/s^2)
    satu_gyro: float = 35.0         # Gyroscope saturation (deg/s)

    # Extrinsics (LiDAR to IMU)
    extrinsic_T: np.ndarray = field(default_factory=lambda: np.zeros(3))
    extrinsic_R: np.ndarray = field(default_factory=lambda: np.eye(3))

    # Gravity
    gravity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))


class SO3:
    """SO(3) Lie group operations for rotation matrices."""

    @staticmethod
    def skew(v: np.ndarray) -> np.ndarray:
        """Create skew-symmetric matrix from 3D vector."""
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])

    @staticmethod
    def exp(omega: np.ndarray) -> np.ndarray:
        """Exponential map: so(3) -> SO(3) (angle-axis to rotation matrix)."""
        theta = np.linalg.norm(omega)
        if theta < 1e-10:
            return np.eye(3) + SO3.skew(omega)

        K = SO3.skew(omega / theta)
        return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * K @ K

    @staticmethod
    def log(R: np.ndarray) -> np.ndarray:
        """Logarithm map: SO(3) -> so(3) (rotation matrix to angle-axis)."""
        trace = np.trace(R)
        if trace >= 3.0 - 1e-10:
            # Near identity
            return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2
        elif trace <= -1.0 + 1e-10:
            # Near 180 degrees
            if R[2, 2] > R[1, 1] and R[2, 2] > R[0, 0]:
                v = np.array([R[0, 2], R[1, 2], 1 + R[2, 2]])
            elif R[1, 1] > R[0, 0]:
                v = np.array([R[0, 1], 1 + R[1, 1], R[2, 1]])
            else:
                v = np.array([1 + R[0, 0], R[1, 0], R[2, 0]])
            v = v / np.linalg.norm(v)
            return np.pi * v
        else:
            theta = np.arccos((trace - 1) / 2)
            return theta / (2 * np.sin(theta)) * np.array([
                R[2, 1] - R[1, 2],
                R[0, 2] - R[2, 0],
                R[1, 0] - R[0, 1]
            ])

    @staticmethod
    def Jr(omega: np.ndarray) -> np.ndarray:
        """Right Jacobian of SO(3)."""
        theta = np.linalg.norm(omega)
        if theta < 1e-10:
            return np.eye(3)

        K = SO3.skew(omega / theta)
        return np.eye(3) - (1 - np.cos(theta)) / theta * K + (theta - np.sin(theta)) / theta * K @ K

    @staticmethod
    def Jr_inv(omega: np.ndarray) -> np.ndarray:
        """Inverse of right Jacobian of SO(3)."""
        theta = np.linalg.norm(omega)
        if theta < 1e-10:
            return np.eye(3)

        K = SO3.skew(omega / theta)
        half_theta = theta / 2
        return np.eye(3) + 0.5 * K + (1 / theta**2 - (1 + np.cos(theta)) / (2 * theta * np.sin(theta))) * K @ K


@dataclass
class PointLIOState:
    """State vector for Point-LIO filter."""
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))      # Position
    rot: np.ndarray = field(default_factory=lambda: np.eye(3))        # Rotation (SO3)
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))      # Velocity
    bg: np.ndarray = field(default_factory=lambda: np.zeros(3))       # Gyro bias
    ba: np.ndarray = field(default_factory=lambda: np.zeros(3))       # Accel bias
    gravity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))

    # Extrinsics (optional calibration)
    offset_R: np.ndarray = field(default_factory=lambda: np.eye(3))   # LiDAR-IMU rotation
    offset_T: np.ndarray = field(default_factory=lambda: np.zeros(3)) # LiDAR-IMU translation

    def copy(self) -> 'PointLIOState':
        """Create a deep copy of state."""
        return PointLIOState(
            pos=self.pos.copy(),
            rot=self.rot.copy(),
            vel=self.vel.copy(),
            bg=self.bg.copy(),
            ba=self.ba.copy(),
            gravity=self.gravity.copy(),
            offset_R=self.offset_R.copy(),
            offset_T=self.offset_T.copy()
        )

    def to_matrix(self) -> np.ndarray:
        """Convert to 4x4 transformation matrix."""
        T = np.eye(4)
        T[:3, :3] = self.rot
        T[:3, 3] = self.pos
        return T


class PointLIO:
    """
    Point-LIO: Tightly-coupled LiDAR-Inertial Odometry using iEKF.

    Implements point-by-point measurement update with:
    - IMU propagation with bias estimation
    - Point-to-plane residuals
    - Iterated Extended Kalman Filter on SO(3) manifold
    """

    def __init__(self, config: Optional[PointLIOConfig] = None):
        self.config = config or PointLIOConfig()

        # State
        self.state = PointLIOState()
        self.state.offset_R = self.config.extrinsic_R.copy()
        self.state.offset_T = self.config.extrinsic_T.copy()
        self.state.gravity = self.config.gravity.copy()

        # Error state covariance (24x24)
        # Order: pos(3), rot(3), offset_R(3), offset_T(3), vel(3), bg(3), ba(3), gravity(3)
        self.P = np.eye(24) * 0.001
        self.P[15:18, 15:18] = np.eye(3) * 0.0001  # bg
        self.P[18:21, 18:21] = np.eye(3) * 0.0001  # ba

        # Local map (KD-tree for neighbor search)
        self.map_points = None
        self.map_tree = None
        self.map_normals = None  # Optional: precomputed normals

        # IMU buffer for interpolation
        self.imu_buffer = []
        self.last_imu_time = None

        # Initialization
        self.initialized = False
        self.init_imu_count = 0
        self.init_acc_sum = np.zeros(3)
        self.init_gyr_sum = np.zeros(3)
        self.init_count_needed = 100

        # Statistics
        self.scan_count = 0
        self.total_points = 0
        self.last_update_time = None

    def _build_Q_matrix(self, dt: float) -> np.ndarray:
        """Build process noise covariance matrix."""
        Q = np.zeros((24, 24))

        # Gyro noise affects rotation
        Q[3:6, 3:6] = np.eye(3) * self.config.gyr_cov * dt**2

        # Accel noise affects velocity
        Q[12:15, 12:15] = np.eye(3) * self.config.acc_cov * dt**2

        # Bias random walk
        Q[15:18, 15:18] = np.eye(3) * self.config.b_gyr_cov * dt**2
        Q[18:21, 18:21] = np.eye(3) * self.config.b_acc_cov * dt**2

        return Q

    def _state_transition_jacobian(self, acc_body: np.ndarray, omega_body: np.ndarray,
                                    dt: float) -> np.ndarray:
        """Compute state transition Jacobian (F_x)."""
        F = np.eye(24)

        # d(pos)/d(vel) = I * dt
        F[0:3, 12:15] = np.eye(3) * dt

        # d(rot)/d(bg) = -I * dt (through omega_body = gyro - bg)
        F[3:6, 15:18] = -np.eye(3) * dt

        # d(vel)/d(rot) = rot^T * [acc - ba]_x * dt
        acc_skew = SO3.skew(acc_body)
        F[12:15, 3:6] = self.state.rot.T @ acc_skew * dt

        # d(vel)/d(ba) = -rot^T * dt
        F[12:15, 18:21] = -self.state.rot.T * dt

        # d(vel)/d(gravity) = I * dt
        F[12:15, 21:24] = np.eye(3) * dt

        return F

    def imu_init(self, acc: np.ndarray, gyro: np.ndarray) -> bool:
        """
        Initialize IMU biases and gravity from stationary measurements.

        Args:
            acc: Accelerometer reading [ax, ay, az] in m/s^2
            gyro: Gyroscope reading [wx, wy, wz] in rad/s

        Returns:
            True if initialization complete
        """
        # Check for saturation
        if np.linalg.norm(acc) > self.config.satu_acc:
            return False
        if np.linalg.norm(gyro) > np.deg2rad(self.config.satu_gyro):
            return False

        self.init_imu_count += 1
        self.init_acc_sum += acc
        self.init_gyr_sum += gyro

        if self.init_imu_count >= self.init_count_needed:
            # Compute mean
            mean_acc = self.init_acc_sum / self.init_imu_count
            mean_gyr = self.init_gyr_sum / self.init_imu_count

            # Gyro bias is mean gyro (should be zero when stationary)
            self.state.bg = mean_gyr.copy()

            # Gravity estimation from accelerometer
            # When stationary, acc measures -gravity in body frame
            gravity_norm = np.linalg.norm(mean_acc)
            if gravity_norm > 8.0:  # Sanity check
                # Align gravity with measured direction
                gravity_dir = -mean_acc / gravity_norm
                self.state.gravity = gravity_dir * 9.81

            self.initialized = True
            print(f"[Point-LIO] Initialized with bg={self.state.bg}, gravity={self.state.gravity}")
            return True

        return False

    def predict(self, acc: np.ndarray, gyro: np.ndarray, dt: float):
        """
        IMU prediction step (state propagation).

        Args:
            acc: Accelerometer reading [ax, ay, az] in m/s^2
            gyro: Gyroscope reading [wx, wy, wz] in rad/s
            dt: Time step in seconds
        """
        if not self.initialized:
            self.imu_init(acc, gyro)
            return

        # Remove biases
        omega_body = gyro - self.state.bg
        acc_body = acc - self.state.ba

        # State propagation
        # Position: p += v * dt + 0.5 * (R^T * a + g) * dt^2
        acc_world = self.state.rot.T @ acc_body + self.state.gravity
        self.state.pos = self.state.pos + self.state.vel * dt + 0.5 * acc_world * dt**2

        # Velocity: v += (R^T * a + g) * dt
        self.state.vel = self.state.vel + acc_world * dt

        # Rotation: R = R * exp(omega * dt)
        dR = SO3.exp(omega_body * dt)
        self.state.rot = self.state.rot @ dR

        # Orthonormalize rotation matrix (prevent drift)
        U, _, Vt = np.linalg.svd(self.state.rot)
        self.state.rot = U @ Vt

        # Covariance propagation: P = F * P * F^T + Q
        F = self._state_transition_jacobian(acc_body, omega_body, dt)
        Q = self._build_Q_matrix(dt)
        self.P = F @ self.P @ F.T + Q

        # Ensure symmetry
        self.P = (self.P + self.P.T) / 2

    def _estimate_plane(self, points: np.ndarray) -> Optional[np.ndarray]:
        """
        Estimate plane parameters from neighbor points.

        Args:
            points: Nx3 array of neighbor points

        Returns:
            Plane parameters [nx, ny, nz, d] or None if failed
        """
        if len(points) < self.config.num_match_points:
            return None

        # Solve: [x y z 1] * [a b c d]^T = 0
        # We use: A * [a b c]^T = -1 where d = 1
        A = points[:, :3]
        b = -np.ones(len(points))

        try:
            # QR decomposition for stability
            Q, R = qr(A, mode='economic')
            n = solve(R, Q.T @ b)

            # Normalize
            norm = np.linalg.norm(n)
            if norm < 1e-6:
                return None

            n_normalized = n / norm
            d = 1.0 / norm

            # Verify all points are close to plane
            errors = np.abs(points @ n_normalized + d)
            if np.max(errors) > self.config.plane_thr:
                return None

            return np.array([n_normalized[0], n_normalized[1], n_normalized[2], d])

        except Exception:
            return None

    def _compute_residual_and_jacobian(self, point_lidar: np.ndarray,
                                        plane: np.ndarray) -> Tuple[float, np.ndarray]:
        """
        Compute point-to-plane residual and measurement Jacobian.

        Args:
            point_lidar: Point in LiDAR frame [x, y, z]
            plane: Plane parameters [nx, ny, nz, d]

        Returns:
            (residual, jacobian) where jacobian is 1x24
        """
        # Transform point: LiDAR -> IMU -> World
        p_imu = self.state.offset_R @ point_lidar + self.state.offset_T
        p_world = self.state.rot.T @ p_imu + self.state.pos

        # Plane normal and offset
        n = plane[:3]
        d = plane[3]

        # Residual: z = -(n^T * p_world + d)
        residual = -(n @ p_world + d)

        # Jacobian h_x (1x24)
        h_x = np.zeros(24)

        # d(residual)/d(pos) = -n
        h_x[0:3] = -n

        # d(residual)/d(rot): need d(p_world)/d(rot)
        # p_world = R^T * p_imu + pos
        # Using error state: d(p_world)/d(delta_theta) = -R^T * [p_imu]_x
        # Then: d(residual)/d(delta_theta) = -n^T * (-R^T * [p_imu]_x) = n^T * R^T * [p_imu]_x
        C = self.state.rot @ n
        h_x[3:6] = SO3.skew(p_imu) @ C

        # d(residual)/d(offset_R): similar derivation
        # h_x[6:9] = (self.state.offset_R.T @ C) @ SO3.skew(point_lidar).T  # If calibrating

        # d(residual)/d(offset_T)
        # h_x[9:12] = -C  # If calibrating

        return residual, h_x

    def update(self, points: np.ndarray, intensities: Optional[np.ndarray] = None) -> Tuple[np.ndarray, dict]:
        """
        LiDAR measurement update using point-to-plane residuals.

        Args:
            points: Nx3 array of points in LiDAR frame
            intensities: Optional Nx1 array of intensities

        Returns:
            (transformed_points, info_dict)
        """
        if not self.initialized:
            return points, {"initialized": False}

        if self.map_tree is None:
            # First scan - build initial map
            self._add_to_map(points)
            self.scan_count += 1
            return self._transform_points(points), {"first_scan": True}

        # Preprocess points
        valid_mask = self._filter_points(points)
        valid_points = points[valid_mask]

        if len(valid_points) < 10:
            return self._transform_points(points), {"too_few_points": True}

        # Downsample for efficiency
        if len(valid_points) > 5000:
            indices = np.random.choice(len(valid_points), 5000, replace=False)
            valid_points = valid_points[indices]

        # Iterated EKF update
        for iteration in range(self.config.max_iterations):
            residuals = []
            jacobians = []

            for point in valid_points:
                # Transform point to world frame
                p_imu = self.state.offset_R @ point + self.state.offset_T
                p_world = self.state.rot.T @ p_imu + self.state.pos

                # Find nearest neighbors in map
                if self.map_tree is None:
                    continue

                distances, indices = self.map_tree.query(p_world, k=self.config.num_match_points)

                if distances[-1] > 1.0:  # Too far from map
                    continue

                neighbor_points = self.map_points[indices]

                # Estimate plane
                plane = self._estimate_plane(neighbor_points)
                if plane is None:
                    continue

                # Point-to-plane criterion (from Point-LIO)
                p_norm = np.linalg.norm(point)
                pd2 = np.abs(plane[:3] @ p_world + plane[3])
                if p_norm < self.config.match_s * pd2**2:
                    continue

                # Compute residual and Jacobian
                z, h_x = self._compute_residual_and_jacobian(point, plane)
                residuals.append(z)
                jacobians.append(h_x)

            if len(residuals) < 5:
                break

            # Stack measurements
            z = np.array(residuals)
            H = np.array(jacobians)

            # Measurement noise
            R = np.eye(len(z)) * self.config.lidar_meas_cov

            # Kalman gain: K = P * H^T * (H * P * H^T + R)^{-1}
            PHt = self.P @ H.T
            S = H @ PHt + R

            try:
                K = PHt @ np.linalg.inv(S)
            except np.linalg.LinAlgError:
                break

            # State update (error state)
            dx = K @ z

            # Apply update using manifold operations
            self._apply_error_state(dx)

            # Covariance update: P = (I - K*H) * P
            I_KH = np.eye(24) - K @ H
            self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
            self.P = (self.P + self.P.T) / 2  # Ensure symmetry

            # Check convergence
            if np.linalg.norm(dx[:6]) < self.config.convergence_thr:
                break

        # Add points to map
        transformed = self._transform_points(valid_points)
        self._add_to_map(transformed)

        self.scan_count += 1
        self.total_points += len(transformed)

        return self._transform_points(points), {
            "scan_count": self.scan_count,
            "map_points": len(self.map_points) if self.map_points is not None else 0,
            "used_points": len(residuals) if 'residuals' in dir() else 0,
            "position": self.state.pos.tolist(),
            "velocity": self.state.vel.tolist()
        }

    def _apply_error_state(self, dx: np.ndarray):
        """Apply error state update using manifold operations."""
        # Position
        self.state.pos += dx[0:3]

        # Rotation (SO3 boxplus)
        delta_rot = SO3.exp(dx[3:6])
        self.state.rot = self.state.rot @ delta_rot

        # Orthonormalize
        U, _, Vt = np.linalg.svd(self.state.rot)
        self.state.rot = U @ Vt

        # Extrinsics (if calibrating)
        # delta_offset_R = SO3.exp(dx[6:9])
        # self.state.offset_R = self.state.offset_R @ delta_offset_R
        # self.state.offset_T += dx[9:12]

        # Velocity
        self.state.vel += dx[12:15]

        # Biases
        self.state.bg += dx[15:18]
        self.state.ba += dx[18:21]

        # Gravity
        self.state.gravity += dx[21:24]

    def _filter_points(self, points: np.ndarray) -> np.ndarray:
        """Filter points by range."""
        ranges = np.linalg.norm(points, axis=1)
        valid = (ranges > self.config.min_range) & (ranges < self.config.max_range)
        return valid

    def _transform_points(self, points: np.ndarray) -> np.ndarray:
        """Transform points from LiDAR frame to world frame."""
        # LiDAR -> IMU
        p_imu = (self.state.offset_R @ points.T).T + self.state.offset_T
        # IMU -> World
        p_world = (self.state.rot.T @ p_imu.T).T + self.state.pos
        return p_world

    def _add_to_map(self, points: np.ndarray, max_map_points: int = 500000):
        """Add points to local map."""
        # Voxel downsampling
        if len(points) > 0:
            # Simple voxel grid downsampling
            voxel_size = self.config.voxel_size
            voxel_indices = np.floor(points / voxel_size).astype(int)
            _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
            downsampled = points[unique_indices]

            if self.map_points is None:
                self.map_points = downsampled
            else:
                self.map_points = np.vstack([self.map_points, downsampled])

                # Limit map size
                if len(self.map_points) > max_map_points:
                    # Keep most recent points
                    self.map_points = self.map_points[-max_map_points:]

            # Rebuild KD-tree
            self.map_tree = cKDTree(self.map_points)

    def get_pose(self) -> np.ndarray:
        """Get current 4x4 pose matrix."""
        return self.state.to_matrix()

    def get_position(self) -> np.ndarray:
        """Get current position."""
        return self.state.pos.copy()

    def get_velocity(self) -> np.ndarray:
        """Get current velocity."""
        return self.state.vel.copy()

    def get_stats(self) -> dict:
        """Get algorithm statistics."""
        return {
            "initialized": self.initialized,
            "scan_count": self.scan_count,
            "map_points": len(self.map_points) if self.map_points is not None else 0,
            "total_points": self.total_points,
            "position": self.state.pos.tolist(),
            "velocity": self.state.vel.tolist(),
            "gyro_bias": self.state.bg.tolist(),
            "accel_bias": self.state.ba.tolist(),
            "gravity": self.state.gravity.tolist()
        }


class PointLIOProcessor:
    """
    High-level processor for Point-LIO integration with web streaming.

    Handles:
    - IMU/LiDAR time synchronization
    - Buffering and batch processing
    - Output formatting for web visualization
    """

    def __init__(self, config: Optional[PointLIOConfig] = None):
        self.lio = PointLIO(config)
        self.imu_buffer = []
        self.last_lidar_time = None
        self.trajectory = []

    def process_imu(self, acc: np.ndarray, gyro: np.ndarray, timestamp: float):
        """
        Process IMU measurement.

        Args:
            acc: Accelerometer [ax, ay, az] in m/s^2
            gyro: Gyroscope [wx, wy, wz] in rad/s
            timestamp: Unix timestamp in seconds
        """
        self.imu_buffer.append({
            'acc': acc.copy(),
            'gyro': gyro.copy(),
            'time': timestamp
        })

        # Limit buffer size
        if len(self.imu_buffer) > 1000:
            self.imu_buffer = self.imu_buffer[-500:]

    def process_scan(self, points: np.ndarray, timestamp: float) -> Tuple[np.ndarray, dict]:
        """
        Process LiDAR scan with IMU integration.

        Args:
            points: Nx3 point cloud in LiDAR frame
            timestamp: Unix timestamp in seconds

        Returns:
            (transformed_points, info_dict)
        """
        # Propagate IMU measurements up to scan time
        if self.last_lidar_time is not None:
            imu_to_process = [m for m in self.imu_buffer
                             if self.last_lidar_time < m['time'] <= timestamp]

            last_time = self.last_lidar_time
            for imu in imu_to_process:
                dt = imu['time'] - last_time
                if dt > 0 and dt < 0.1:  # Sanity check
                    self.lio.predict(imu['acc'], imu['gyro'], dt)
                last_time = imu['time']

        self.last_lidar_time = timestamp

        # LiDAR update
        transformed, info = self.lio.update(points)

        # Record trajectory
        self.trajectory.append({
            'time': timestamp,
            'position': self.lio.get_position().tolist(),
            'pose': self.lio.get_pose().tolist()
        })

        # Keep trajectory manageable
        if len(self.trajectory) > 10000:
            self.trajectory = self.trajectory[-5000:]

        return transformed, info

    def get_trajectory(self) -> List[dict]:
        """Get recorded trajectory."""
        return self.trajectory

    def get_stats(self) -> dict:
        """Get processor statistics."""
        stats = self.lio.get_stats()
        stats['trajectory_length'] = len(self.trajectory)
        stats['imu_buffer_size'] = len(self.imu_buffer)
        return stats


# Convenience function for integration with existing SLAM system
def create_point_lio(voxel_size: float = 0.3,
                     max_range: float = 50.0,
                     extrinsic_T: Optional[np.ndarray] = None,
                     extrinsic_R: Optional[np.ndarray] = None) -> PointLIOProcessor:
    """
    Create Point-LIO processor with common settings.

    Args:
        voxel_size: Downsampling voxel size in meters
        max_range: Maximum point range in meters
        extrinsic_T: LiDAR-IMU translation [x, y, z]
        extrinsic_R: LiDAR-IMU rotation matrix (3x3)

    Returns:
        Configured PointLIOProcessor
    """
    config = PointLIOConfig(
        voxel_size=voxel_size,
        max_range=max_range,
        extrinsic_T=extrinsic_T if extrinsic_T is not None else np.zeros(3),
        extrinsic_R=extrinsic_R if extrinsic_R is not None else np.eye(3)
    )
    return PointLIOProcessor(config)
