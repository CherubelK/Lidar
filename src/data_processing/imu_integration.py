"""
IMU Integration for LiDAR Motion Compensation

Based on ORB-SLAM3's visual-inertial approach:
- Tightly-coupled IMU integration for motion prediction
- Point cloud deskewing using angular velocity
- Gravity-aligned coordinate frame

This module provides:
1. IMU state tracking (orientation, velocity, position)
2. Motion prediction between LiDAR frames
3. Point cloud deskewing for handheld scanning
"""

import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass, field
from collections import deque
import time
import logging

logger = logging.getLogger(__name__)


@dataclass
class IMUState:
    """Current IMU state estimate"""
    timestamp: float = 0.0

    # Orientation as quaternion [w, x, y, z]
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))

    # Angular velocity (rad/s)
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Linear acceleration (m/s^2) - gravity compensated
    linear_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Estimated velocity (m/s) - integrated from acceleration
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Estimated position delta since last reset
    position_delta: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass
class IMUMeasurement:
    """Single IMU measurement"""
    timestamp: float
    quaternion: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray


class IMUIntegration:
    """
    IMU integration for motion compensation in LiDAR SLAM.

    Implements:
    - Quaternion-based orientation tracking
    - Velocity estimation via acceleration integration
    - Point cloud deskewing using angular velocity
    - Motion prediction for ICP initial guess
    """

    def __init__(self, gravity: float = 9.81, buffer_size: int = 100):
        """
        Initialize IMU integration.

        Args:
            gravity: Gravity magnitude (m/s^2)
            buffer_size: Number of IMU measurements to buffer
        """
        self.gravity = gravity
        self.gravity_vector = np.array([0, 0, -gravity])  # Assuming Z-up

        # State
        self.state = IMUState()
        self.initialized = False
        self.last_update_time = None

        # Measurement buffer for interpolation
        self.measurements = deque(maxlen=buffer_size)

        # Bias estimates (can be calibrated)
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)

        # Reference orientation (set at initialization)
        self.reference_quaternion = None

        logger.info("IMU Integration initialized")

    def update(self, imu_data: dict, timestamp: Optional[float] = None) -> IMUState:
        """
        Update IMU state with new measurement.

        Args:
            imu_data: Dictionary with 'quaternion', 'angular_velocity', 'linear_acceleration'
            timestamp: Measurement timestamp (uses current time if None)

        Returns:
            Updated IMU state
        """
        if timestamp is None:
            timestamp = time.time()

        # Extract measurements
        quaternion = np.array(imu_data['quaternion'])
        angular_velocity = np.array(imu_data['angular_velocity']) - self.gyro_bias
        linear_acceleration = np.array(imu_data['linear_acceleration']) - self.accel_bias

        # Store measurement
        measurement = IMUMeasurement(
            timestamp=timestamp,
            quaternion=quaternion,
            angular_velocity=angular_velocity,
            linear_acceleration=linear_acceleration
        )
        self.measurements.append(measurement)

        # Initialize on first measurement
        if not self.initialized:
            self._initialize(measurement)
            return self.state

        # Calculate dt
        dt = timestamp - self.last_update_time
        if dt <= 0 or dt > 1.0:  # Sanity check
            dt = 0.01  # Default to 100Hz

        # Update orientation (use sensor quaternion directly or integrate gyro)
        self.state.quaternion = quaternion.copy()
        self.state.angular_velocity = angular_velocity.copy()

        # Remove gravity from acceleration (in world frame)
        rotation_matrix = self._quaternion_to_rotation_matrix(quaternion)
        accel_world = rotation_matrix @ linear_acceleration
        accel_world_no_gravity = accel_world - self.gravity_vector

        self.state.linear_acceleration = accel_world_no_gravity

        # Integrate velocity (with simple dampening to prevent drift)
        damping = 0.98  # Slight damping to reduce drift
        self.state.velocity = damping * self.state.velocity + accel_world_no_gravity * dt

        # Integrate position delta
        self.state.position_delta += self.state.velocity * dt

        self.state.timestamp = timestamp
        self.last_update_time = timestamp

        return self.state

    def _initialize(self, measurement: IMUMeasurement) -> None:
        """Initialize IMU state from first measurement."""
        self.state.quaternion = measurement.quaternion.copy()
        self.state.angular_velocity = measurement.angular_velocity.copy()
        self.state.linear_acceleration = measurement.linear_acceleration.copy()
        self.state.velocity = np.zeros(3)
        self.state.position_delta = np.zeros(3)
        self.state.timestamp = measurement.timestamp

        self.reference_quaternion = measurement.quaternion.copy()
        self.last_update_time = measurement.timestamp
        self.initialized = True

        logger.info("IMU initialized with reference orientation")

    def get_motion_prediction(self, dt: float) -> np.ndarray:
        """
        Predict motion for the next dt seconds.

        Returns a 4x4 transformation matrix predicting the motion.
        Used as initial guess for ICP registration.

        Args:
            dt: Time interval to predict (seconds)

        Returns:
            4x4 transformation matrix
        """
        if not self.initialized:
            return np.eye(4)

        # Predict rotation from angular velocity
        omega = self.state.angular_velocity
        angle = np.linalg.norm(omega) * dt

        if angle > 1e-6:
            axis = omega / np.linalg.norm(omega)
            # Rodrigues' rotation formula
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])
            R_delta = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
        else:
            R_delta = np.eye(3)

        # Predict translation from velocity
        t_delta = self.state.velocity * dt

        # Build transformation matrix
        T = np.eye(4)
        T[:3, :3] = R_delta
        T[:3, 3] = t_delta

        return T

    def deskew_point_cloud(self, points: np.ndarray,
                          scan_duration: float = 0.1) -> np.ndarray:
        """
        Deskew point cloud using IMU angular velocity.

        During a LiDAR scan, the sensor rotates. This function compensates
        for that rotation by "unwinding" each point based on when it was
        captured during the scan.

        Args:
            points: Nx3 point cloud
            scan_duration: Duration of the scan in seconds

        Returns:
            Deskewed Nx3 point cloud
        """
        if not self.initialized or len(points) == 0:
            return points

        n_points = len(points)
        omega = self.state.angular_velocity
        omega_norm = np.linalg.norm(omega)

        # If angular velocity is negligible, no deskewing needed
        if omega_norm < 0.01:  # ~0.5 deg/s threshold
            return points

        # Assume points are captured linearly over scan duration
        # Point i was captured at time t_i = i/n * scan_duration
        point_times = np.linspace(0, scan_duration, n_points)

        # Reference time is end of scan (most recent)
        ref_time = scan_duration

        # Deskew each point
        deskewed = np.zeros_like(points)

        for i, (point, t) in enumerate(zip(points, point_times)):
            dt = ref_time - t  # Time from this point to reference
            angle = omega_norm * dt

            if angle > 1e-6:
                axis = omega / omega_norm
                # Rotate point back to reference time
                K = np.array([
                    [0, -axis[2], axis[1]],
                    [axis[2], 0, -axis[0]],
                    [-axis[1], axis[0], 0]
                ])
                R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
                deskewed[i] = R @ point
            else:
                deskewed[i] = point

        logger.debug(f"Deskewed {n_points} points, omega={omega_norm:.3f} rad/s")
        return deskewed

    def deskew_point_cloud_fast(self, points: np.ndarray,
                                scan_duration: float = 0.1) -> np.ndarray:
        """
        Fast vectorized point cloud deskewing.

        Uses small-angle approximation for efficiency when angular
        velocity is moderate.

        Args:
            points: Nx3 point cloud
            scan_duration: Duration of the scan in seconds

        Returns:
            Deskewed Nx3 point cloud
        """
        if not self.initialized or len(points) == 0:
            return points

        omega = self.state.angular_velocity
        omega_norm = np.linalg.norm(omega)

        # If angular velocity is negligible, no deskewing needed
        if omega_norm < 0.01:
            return points

        n_points = len(points)

        # Time offset for each point (0 = start of scan, scan_duration = end)
        # Reference to end of scan
        dt = np.linspace(-scan_duration, 0, n_points).reshape(-1, 1)

        # Small angle approximation: R ≈ I + [omega]_x * dt
        # For each point: p' = p + (omega x p) * dt
        omega_cross_p = np.cross(omega, points)  # Nx3

        deskewed = points + omega_cross_p * dt

        return deskewed

    def get_relative_rotation(self) -> np.ndarray:
        """
        Get rotation matrix from reference orientation to current.

        Returns:
            3x3 rotation matrix
        """
        if self.reference_quaternion is None:
            return np.eye(3)

        # Compute relative quaternion: q_rel = q_current * q_ref^-1
        q_ref_inv = self._quaternion_conjugate(self.reference_quaternion)
        q_rel = self._quaternion_multiply(self.state.quaternion, q_ref_inv)

        return self._quaternion_to_rotation_matrix(q_rel)

    def reset_position(self) -> None:
        """Reset position delta to zero (keep orientation)."""
        self.state.position_delta = np.zeros(3)
        self.state.velocity = np.zeros(3)

    def get_transform(self) -> np.ndarray:
        """
        Get current transformation from reference frame.

        Returns:
            4x4 transformation matrix
        """
        T = np.eye(4)
        T[:3, :3] = self.get_relative_rotation()
        T[:3, 3] = self.state.position_delta
        return T

    def calibrate_bias(self, duration: float = 2.0) -> bool:
        """
        Calibrate gyro and accelerometer biases from stationary data.

        Should be called when the sensor is stationary.

        Args:
            duration: How many seconds of data to use

        Returns:
            True if calibration successful
        """
        if len(self.measurements) < 10:
            logger.warning("Not enough measurements for bias calibration")
            return False

        # Use recent measurements
        cutoff_time = time.time() - duration
        recent = [m for m in self.measurements if m.timestamp > cutoff_time]

        if len(recent) < 10:
            logger.warning("Not enough recent measurements for calibration")
            return False

        # Average gyro readings (should be zero when stationary)
        gyro_readings = np.array([m.angular_velocity for m in recent])
        self.gyro_bias = np.mean(gyro_readings, axis=0)

        # Average accel readings (should be gravity vector when stationary)
        accel_readings = np.array([m.linear_acceleration for m in recent])
        accel_mean = np.mean(accel_readings, axis=0)

        # Bias is difference from expected gravity
        expected_gravity = np.array([0, 0, self.gravity])  # Assuming Z-up
        self.accel_bias = accel_mean - expected_gravity

        logger.info(f"IMU bias calibrated: gyro={self.gyro_bias}, accel={self.accel_bias}")
        return True

    # Quaternion utilities
    @staticmethod
    def _quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""
        w, x, y, z = q

        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])

    @staticmethod
    def _quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    @staticmethod
    def _quaternion_conjugate(q: np.ndarray) -> np.ndarray:
        """Compute quaternion conjugate (inverse for unit quaternions)."""
        return np.array([q[0], -q[1], -q[2], -q[3]])


class IMUPreintegration:
    """
    IMU preintegration for efficient pose graph optimization.

    Based on the preintegration theory from:
    - Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
    - ORB-SLAM3's IMU handling

    Preintegrates IMU measurements between keyframes to create
    relative motion constraints without re-integrating when
    linearization point changes.
    """

    def __init__(self):
        """Initialize preintegration."""
        self.reset()

    def reset(self):
        """Reset preintegration state."""
        self.delta_R = np.eye(3)  # Rotation
        self.delta_v = np.zeros(3)  # Velocity
        self.delta_p = np.zeros(3)  # Position
        self.dt_sum = 0.0
        self.measurements = []

    def integrate(self, gyro: np.ndarray, accel: np.ndarray, dt: float) -> None:
        """
        Integrate a single IMU measurement.

        Args:
            gyro: Angular velocity (rad/s)
            accel: Linear acceleration (m/s^2)
            dt: Time step (seconds)
        """
        # Store measurement
        self.measurements.append((gyro.copy(), accel.copy(), dt))

        # Rotation update
        omega_norm = np.linalg.norm(gyro)
        if omega_norm > 1e-6:
            axis = gyro / omega_norm
            angle = omega_norm * dt
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])
            dR = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
        else:
            dR = np.eye(3)

        # Velocity update (in body frame, then rotated)
        self.delta_v += self.delta_R @ accel * dt

        # Position update
        self.delta_p += self.delta_v * dt + 0.5 * self.delta_R @ accel * dt * dt

        # Rotation update
        self.delta_R = self.delta_R @ dR

        self.dt_sum += dt

    def get_delta_transform(self) -> np.ndarray:
        """
        Get the preintegrated transformation.

        Returns:
            4x4 transformation matrix
        """
        T = np.eye(4)
        T[:3, :3] = self.delta_R
        T[:3, 3] = self.delta_p
        return T


if __name__ == "__main__":
    # Test IMU integration
    logging.basicConfig(level=logging.INFO)
    print("=== IMU Integration Test ===\n")

    imu = IMUIntegration()

    # Simulate IMU data (rotating around Z axis)
    print("Simulating rotation around Z axis...")
    for i in range(100):
        fake_imu = {
            'quaternion': np.array([1, 0, 0, 0]),  # Identity
            'angular_velocity': np.array([0, 0, 0.5]),  # 0.5 rad/s around Z
            'linear_acceleration': np.array([0, 0, 9.81])  # Gravity
        }
        state = imu.update(fake_imu, timestamp=i * 0.01)

    print(f"Angular velocity: {state.angular_velocity}")
    print(f"Velocity estimate: {state.velocity}")

    # Test deskewing
    print("\nTesting point cloud deskewing...")
    points = np.random.randn(1000, 3)
    deskewed = imu.deskew_point_cloud_fast(points, scan_duration=0.1)

    diff = np.linalg.norm(deskewed - points, axis=1).mean()
    print(f"Average point displacement: {diff:.4f}m")

    # Test motion prediction
    print("\nTesting motion prediction...")
    T_pred = imu.get_motion_prediction(dt=0.1)
    print(f"Predicted transform:\n{T_pred}")