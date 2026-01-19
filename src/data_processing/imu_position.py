"""
IMU-Based Position Estimation for Unitree L2 LiDAR

Calculates position using:
- Quaternion (orientation)
- Angular velocity (gyroscope)
- Linear acceleration (accelerometer)

Based on the Unitree L2 IMU data format:
- Quaternion: [w, x, y, z] or [x, y, z, w] depending on SDK version
- Angular velocity: [wx, wy, wz] in rad/s
- Linear acceleration: [ax, ay, az] in m/s^2

IMU coordinate system offset from LiDAR: [-0.007698, -0.014655, 0.00667] meters
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import time


@dataclass
class IMUPositionConfig:
    """Configuration for IMU position estimation."""
    # Gravity magnitude
    gravity_magnitude: float = 9.81

    # Initial gravity direction (will be estimated from IMU)
    gravity_init: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))

    # IMU to LiDAR offset (from Unitree documentation)
    imu_to_lidar_offset: np.ndarray = field(default_factory=lambda: np.array([-0.007698, -0.014655, 0.00667]))

    # Noise parameters for filtering
    accel_noise_std: float = 0.1      # m/s^2
    gyro_noise_std: float = 0.01      # rad/s
    accel_bias_std: float = 0.001     # m/s^2 random walk
    gyro_bias_std: float = 0.0001     # rad/s random walk

    # Position estimation settings
    use_gravity_compensation: bool = True
    use_bias_estimation: bool = True

    # Velocity decay (for drift mitigation when stationary)
    velocity_decay: float = 0.995  # Applied per update when low motion detected
    stationary_threshold: float = 0.5  # m/s^2 acceleration variance threshold


class Quaternion:
    """Quaternion operations for orientation handling."""

    @staticmethod
    def normalize(q: np.ndarray) -> np.ndarray:
        """Normalize quaternion to unit length."""
        norm = np.linalg.norm(q)
        if norm < 1e-10:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / norm

    @staticmethod
    def to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        """
        Convert quaternion to 3x3 rotation matrix.

        Args:
            q: Quaternion [w, x, y, z]

        Returns:
            3x3 rotation matrix
        """
        q = Quaternion.normalize(q)
        w, x, y, z = q

        # Rotation matrix from quaternion
        R = np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - w*z),       2*(x*z + w*y)],
            [2*(x*y + w*z),           1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y),           2*(y*z + w*x),       1 - 2*(x**2 + y**2)]
        ])
        return R

    @staticmethod
    def from_rotation_matrix(R: np.ndarray) -> np.ndarray:
        """
        Convert 3x3 rotation matrix to quaternion.

        Args:
            R: 3x3 rotation matrix

        Returns:
            Quaternion [w, x, y, z]
        """
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return Quaternion.normalize(np.array([w, x, y, z]))

    @staticmethod
    def multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """
        Multiply two quaternions (q1 * q2).

        Args:
            q1, q2: Quaternions [w, x, y, z]

        Returns:
            Product quaternion
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    @staticmethod
    def conjugate(q: np.ndarray) -> np.ndarray:
        """Return quaternion conjugate (inverse for unit quaternion)."""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    @staticmethod
    def rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """
        Rotate vector v by quaternion q.

        Args:
            q: Quaternion [w, x, y, z]
            v: Vector [x, y, z]

        Returns:
            Rotated vector
        """
        # Use rotation matrix (more numerically stable)
        R = Quaternion.to_rotation_matrix(q)
        return R @ v

    @staticmethod
    def from_angular_velocity(omega: np.ndarray, dt: float) -> np.ndarray:
        """
        Create quaternion from angular velocity and time step.

        Args:
            omega: Angular velocity [wx, wy, wz] in rad/s
            dt: Time step in seconds

        Returns:
            Quaternion representing rotation
        """
        angle = np.linalg.norm(omega) * dt
        if angle < 1e-10:
            return np.array([1.0, 0.0, 0.0, 0.0])

        axis = omega / np.linalg.norm(omega)
        half_angle = angle / 2

        return np.array([
            np.cos(half_angle),
            axis[0] * np.sin(half_angle),
            axis[1] * np.sin(half_angle),
            axis[2] * np.sin(half_angle)
        ])

    @staticmethod
    def slerp(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """
        Spherical linear interpolation between quaternions.

        Args:
            q1, q2: Quaternions
            t: Interpolation factor [0, 1]

        Returns:
            Interpolated quaternion
        """
        q1 = Quaternion.normalize(q1)
        q2 = Quaternion.normalize(q2)

        dot = np.dot(q1, q2)

        # If negative dot, negate one quaternion to take shorter path
        if dot < 0:
            q2 = -q2
            dot = -dot

        if dot > 0.9995:
            # Linear interpolation for very close quaternions
            result = q1 + t * (q2 - q1)
            return Quaternion.normalize(result)

        theta_0 = np.arccos(dot)
        theta = theta_0 * t

        q_perp = q2 - q1 * dot
        q_perp = Quaternion.normalize(q_perp)

        return q1 * np.cos(theta) + q_perp * np.sin(theta)


class IMUPositionEstimator:
    """
    Estimates position from IMU data using dead reckoning with gravity compensation.

    Algorithm:
    1. Use quaternion to get orientation (body to world frame)
    2. Transform acceleration from body frame to world frame
    3. Subtract gravity to get linear acceleration
    4. Double integrate to get velocity and position
    """

    def __init__(self, config: Optional[IMUPositionConfig] = None):
        self.config = config or IMUPositionConfig()

        # State
        self.position = np.zeros(3)           # [x, y, z] in world frame
        self.velocity = np.zeros(3)           # [vx, vy, vz] in world frame
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z] quaternion

        # Bias estimation
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

        # Gravity in world frame (estimated or known)
        self.gravity_world = self.config.gravity_init.copy()

        # Initialization
        self.initialized = False
        self.init_samples = []
        self.init_count_needed = 50

        # Timing
        self.last_time = None

        # Statistics
        self.update_count = 0
        self.trajectory = []

        # Motion detection
        self.recent_accels = []
        self.max_recent = 20

    def initialize_from_stationary(self, accel: np.ndarray, gyro: np.ndarray) -> bool:
        """
        Initialize biases and gravity from stationary IMU measurements.

        Args:
            accel: Linear acceleration [ax, ay, az] in m/s^2
            gyro: Angular velocity [wx, wy, wz] in rad/s

        Returns:
            True if initialization complete
        """
        self.init_samples.append({
            'accel': accel.copy(),
            'gyro': gyro.copy()
        })

        if len(self.init_samples) >= self.init_count_needed:
            # Compute mean values
            mean_accel = np.mean([s['accel'] for s in self.init_samples], axis=0)
            mean_gyro = np.mean([s['gyro'] for s in self.init_samples], axis=0)

            # Gyro bias is mean gyro (should be zero when stationary)
            self.gyro_bias = mean_gyro.copy()

            # Gravity estimation: when stationary, accelerometer measures -gravity
            # in body frame, so gravity_body = -mean_accel
            gravity_body = -mean_accel
            gravity_norm = np.linalg.norm(gravity_body)

            if gravity_norm > 8.0 and gravity_norm < 11.0:  # Sanity check
                # Normalize to known gravity magnitude
                self.gravity_world = gravity_body * (self.config.gravity_magnitude / gravity_norm)

                # Estimate initial accel bias (residual after gravity)
                expected_accel = -self.gravity_world
                self.accel_bias = mean_accel - expected_accel

                self.initialized = True
                print(f"[IMU Position] Initialized:")
                print(f"  Gyro bias: {self.gyro_bias}")
                print(f"  Accel bias: {self.accel_bias}")
                print(f"  Gravity: {self.gravity_world}")
                return True

        return False

    def update_from_quaternion(self, quaternion: np.ndarray, accel: np.ndarray,
                                gyro: np.ndarray, timestamp: float) -> np.ndarray:
        """
        Update position estimate using quaternion orientation directly.

        This uses the IMU's onboard orientation estimate (from sensor fusion)
        combined with acceleration for position integration.

        Args:
            quaternion: Orientation [w, x, y, z] or [x, y, z, w]
            accel: Linear acceleration [ax, ay, az] in m/s^2 (body frame)
            gyro: Angular velocity [wx, wy, wz] in rad/s
            timestamp: Time in seconds

        Returns:
            Current position estimate [x, y, z]
        """
        # Handle initialization
        if not self.initialized:
            if self.initialize_from_stationary(accel, gyro):
                self.last_time = timestamp
            return self.position.copy()

        # Calculate dt
        if self.last_time is None:
            self.last_time = timestamp
            return self.position.copy()

        dt = timestamp - self.last_time
        if dt <= 0 or dt > 0.5:  # Sanity check
            self.last_time = timestamp
            return self.position.copy()

        self.last_time = timestamp

        # Determine quaternion format and normalize
        # Unitree uses [x, y, z, w] format based on SDK examples
        # Convert to [w, x, y, z] for our calculations
        if len(quaternion) == 4:
            # Check if first element could be w (typically close to 1 for small rotations)
            # or if it's in [x, y, z, w] format
            if abs(quaternion[3]) > 0.5:  # Likely [x, y, z, w] format
                q = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])
            else:
                q = quaternion.copy()
        else:
            q = self.orientation.copy()

        q = Quaternion.normalize(q)
        self.orientation = q

        # Get rotation matrix from quaternion (body to world)
        R_body_to_world = Quaternion.to_rotation_matrix(q)

        # Remove biases from measurements
        accel_corrected = accel - self.accel_bias
        gyro_corrected = gyro - self.gyro_bias

        # Transform acceleration to world frame
        accel_world = R_body_to_world @ accel_corrected

        # Gravity compensation
        if self.config.use_gravity_compensation:
            # Remove gravity component
            accel_linear = accel_world - self.gravity_world
        else:
            accel_linear = accel_world

        # Motion detection for drift mitigation
        self.recent_accels.append(accel_linear.copy())
        if len(self.recent_accels) > self.max_recent:
            self.recent_accels.pop(0)

        accel_variance = np.var(self.recent_accels, axis=0).sum() if len(self.recent_accels) > 5 else 1.0

        # Integration with trapezoidal rule
        # v = v + a * dt
        # p = p + v * dt + 0.5 * a * dt^2
        self.velocity = self.velocity + accel_linear * dt
        self.position = self.position + self.velocity * dt + 0.5 * accel_linear * dt**2

        # Apply velocity decay when stationary (drift mitigation)
        if accel_variance < self.config.stationary_threshold:
            self.velocity *= self.config.velocity_decay

        # Record trajectory
        self.update_count += 1
        if self.update_count % 10 == 0:  # Subsample for storage
            self.trajectory.append({
                'time': timestamp,
                'position': self.position.copy(),
                'velocity': self.velocity.copy(),
                'orientation': q.copy()
            })

            # Limit trajectory length
            if len(self.trajectory) > 10000:
                self.trajectory = self.trajectory[-5000:]

        return self.position.copy()

    def update_from_gyro(self, accel: np.ndarray, gyro: np.ndarray,
                          timestamp: float) -> np.ndarray:
        """
        Update position estimate using gyro integration for orientation.

        Use this when quaternion is not available or not trusted.

        Args:
            accel: Linear acceleration [ax, ay, az] in m/s^2 (body frame)
            gyro: Angular velocity [wx, wy, wz] in rad/s
            timestamp: Time in seconds

        Returns:
            Current position estimate [x, y, z]
        """
        # Handle initialization
        if not self.initialized:
            if self.initialize_from_stationary(accel, gyro):
                self.last_time = timestamp
            return self.position.copy()

        # Calculate dt
        if self.last_time is None:
            self.last_time = timestamp
            return self.position.copy()

        dt = timestamp - self.last_time
        if dt <= 0 or dt > 0.5:
            self.last_time = timestamp
            return self.position.copy()

        self.last_time = timestamp

        # Remove biases
        accel_corrected = accel - self.accel_bias
        gyro_corrected = gyro - self.gyro_bias

        # Integrate orientation using gyro
        dq = Quaternion.from_angular_velocity(gyro_corrected, dt)
        self.orientation = Quaternion.multiply(self.orientation, dq)
        self.orientation = Quaternion.normalize(self.orientation)

        # Transform acceleration to world frame
        R_body_to_world = Quaternion.to_rotation_matrix(self.orientation)
        accel_world = R_body_to_world @ accel_corrected

        # Gravity compensation
        if self.config.use_gravity_compensation:
            accel_linear = accel_world - self.gravity_world
        else:
            accel_linear = accel_world

        # Integration
        self.velocity = self.velocity + accel_linear * dt
        self.position = self.position + self.velocity * dt + 0.5 * accel_linear * dt**2

        # Motion detection and drift mitigation
        self.recent_accels.append(accel_linear.copy())
        if len(self.recent_accels) > self.max_recent:
            self.recent_accels.pop(0)

        accel_variance = np.var(self.recent_accels, axis=0).sum() if len(self.recent_accels) > 5 else 1.0
        if accel_variance < self.config.stationary_threshold:
            self.velocity *= self.config.velocity_decay

        self.update_count += 1
        return self.position.copy()

    def reset(self):
        """Reset position estimate to origin."""
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.trajectory = []
        print("[IMU Position] Reset to origin")

    def get_pose(self) -> np.ndarray:
        """
        Get current 4x4 pose matrix.

        Returns:
            4x4 transformation matrix (world frame)
        """
        T = np.eye(4)
        T[:3, :3] = Quaternion.to_rotation_matrix(self.orientation)
        T[:3, 3] = self.position
        return T

    def get_state(self) -> dict:
        """Get current state as dictionary."""
        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'orientation': self.orientation.copy(),
            'rotation_matrix': Quaternion.to_rotation_matrix(self.orientation),
            'euler_angles': self._quaternion_to_euler(self.orientation),
            'initialized': self.initialized,
            'update_count': self.update_count
        }

    def _quaternion_to_euler(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        w, x, y, z = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

    def get_trajectory(self) -> List[dict]:
        """Get recorded trajectory."""
        return self.trajectory


class IMULiDARFusion:
    """
    Fuses IMU position estimates with LiDAR odometry for improved accuracy.

    Uses IMU for:
    - High-frequency pose prediction between LiDAR scans
    - Motion model for scan matching initialization
    - Drift correction when LiDAR matches are poor

    Uses LiDAR for:
    - Absolute position correction
    - IMU bias estimation updates
    - Loop closure detection
    """

    def __init__(self, config: Optional[IMUPositionConfig] = None):
        self.imu_estimator = IMUPositionEstimator(config)

        # Fusion state
        self.fused_position = np.zeros(3)
        self.fused_velocity = np.zeros(3)
        self.fused_orientation = np.array([1.0, 0.0, 0.0, 0.0])

        # LiDAR correction
        self.last_lidar_position = None
        self.last_lidar_time = None

        # Fusion weights
        self.imu_weight = 0.3  # Weight for IMU prediction
        self.lidar_weight = 0.7  # Weight for LiDAR correction

    def update_imu(self, quaternion: np.ndarray, accel: np.ndarray,
                   gyro: np.ndarray, timestamp: float) -> np.ndarray:
        """
        Update with IMU measurement (high frequency).

        Returns:
            Fused position estimate
        """
        imu_position = self.imu_estimator.update_from_quaternion(
            quaternion, accel, gyro, timestamp
        )

        # If no LiDAR correction yet, use pure IMU
        if self.last_lidar_position is None:
            self.fused_position = imu_position
            self.fused_velocity = self.imu_estimator.velocity.copy()
            self.fused_orientation = self.imu_estimator.orientation.copy()
        else:
            # Blend IMU prediction with last known good position
            dt = timestamp - self.last_lidar_time if self.last_lidar_time else 0

            # IMU-predicted position from last LiDAR position
            predicted = self.last_lidar_position + self.fused_velocity * dt

            # Blend with pure IMU estimate (more IMU weight as time increases)
            blend_factor = min(dt * 10, 0.9)  # Increase IMU trust over time
            self.fused_position = (1 - blend_factor) * predicted + blend_factor * imu_position

        return self.fused_position.copy()

    def update_lidar(self, lidar_position: np.ndarray, timestamp: float) -> np.ndarray:
        """
        Update with LiDAR odometry correction (low frequency).

        Args:
            lidar_position: Position from LiDAR odometry [x, y, z]
            timestamp: Time in seconds

        Returns:
            Corrected fused position
        """
        if self.last_lidar_position is not None:
            # Compute velocity from LiDAR
            dt = timestamp - self.last_lidar_time if self.last_lidar_time else 1.0
            if dt > 0.01:
                lidar_velocity = (lidar_position - self.last_lidar_position) / dt

                # Update fused velocity (blend with IMU)
                self.fused_velocity = (self.lidar_weight * lidar_velocity +
                                       self.imu_weight * self.imu_estimator.velocity)

        # Correct position
        self.fused_position = lidar_position.copy()
        self.last_lidar_position = lidar_position.copy()
        self.last_lidar_time = timestamp

        # Optionally reset IMU drift
        position_error = lidar_position - self.imu_estimator.position
        if np.linalg.norm(position_error) > 1.0:  # Large drift detected
            # Apply partial correction to IMU estimator
            self.imu_estimator.position += 0.5 * position_error
            self.imu_estimator.velocity *= 0.9  # Reduce velocity confidence

        return self.fused_position.copy()

    def get_state(self) -> dict:
        """Get fused state."""
        return {
            'fused_position': self.fused_position.copy(),
            'fused_velocity': self.fused_velocity.copy(),
            'fused_orientation': self.fused_orientation.copy(),
            'imu_state': self.imu_estimator.get_state(),
            'last_lidar_position': self.last_lidar_position.copy() if self.last_lidar_position is not None else None
        }


def calculate_position_from_imu(imu_data: dict, estimator: IMUPositionEstimator,
                                 timestamp: float) -> dict:
    """
    Convenience function to calculate position from IMU data dictionary.

    Args:
        imu_data: Dictionary with 'quaternion', 'angular_velocity', 'linear_acceleration'
        estimator: IMUPositionEstimator instance
        timestamp: Current timestamp

    Returns:
        Dictionary with position, velocity, orientation
    """
    quaternion = imu_data.get('quaternion', np.array([1, 0, 0, 0]))
    angular_velocity = imu_data.get('angular_velocity', np.zeros(3))
    linear_acceleration = imu_data.get('linear_acceleration', np.zeros(3))

    position = estimator.update_from_quaternion(
        quaternion, linear_acceleration, angular_velocity, timestamp
    )

    state = estimator.get_state()
    state['position'] = position

    return state