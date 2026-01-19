#!/usr/bin/env python3
"""
IMU Position Estimation Demo for Unitree L2 LiDAR

Demonstrates calculating position using the IMU's:
- Quaternion (orientation)
- Angular velocity (gyroscope)
- Linear acceleration (accelerometer)

Usage:
    python examples/imu_position_demo.py

The script will:
1. Connect to the Unitree L2 LiDAR
2. Collect IMU data for initialization (keep sensor stationary)
3. Track position as you move the sensor
4. Display real-time position estimates
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lidar_interface.unitree_l2_udp import UnitreeL2UDP, UnitreeL2Config
from src.data_processing.imu_position import (
    IMUPositionEstimator, IMUPositionConfig, IMULiDARFusion,
    Quaternion, calculate_position_from_imu
)


def format_vector(v, decimals=3):
    """Format vector for display."""
    return f"[{v[0]:+.{decimals}f}, {v[1]:+.{decimals}f}, {v[2]:+.{decimals}f}]"


def format_quaternion(q, decimals=4):
    """Format quaternion for display."""
    return f"[w={q[0]:+.{decimals}f}, x={q[1]:+.{decimals}f}, y={q[2]:+.{decimals}f}, z={q[3]:+.{decimals}f}]"


def run_imu_position_demo():
    """Run the IMU position estimation demo."""
    print("=" * 60)
    print("  IMU Position Estimation Demo - Unitree L2 LiDAR")
    print("=" * 60)
    print()

    # Initialize LiDAR connection
    config = UnitreeL2Config()
    lidar = UnitreeL2UDP(config)

    print("Connecting to Unitree L2 LiDAR...")
    if not lidar.connect():
        print("Failed to connect to LiDAR. Check network settings.")
        return

    print("Connected!")
    print()

    # Initialize IMU position estimator
    imu_config = IMUPositionConfig(
        gravity_magnitude=9.81,
        use_gravity_compensation=True,
        use_bias_estimation=True,
        velocity_decay=0.995,
        stationary_threshold=0.3
    )
    estimator = IMUPositionEstimator(imu_config)

    print("=" * 60)
    print("  INITIALIZATION PHASE")
    print("  Keep the sensor STATIONARY for calibration...")
    print("=" * 60)
    print()

    # Initialization phase - collect stationary samples
    init_start = time.time()
    imu_count = 0

    while not estimator.initialized:
        packet = lidar.receive_packet()
        if packet is None:
            continue

        result = lidar.parse_packet(packet)

        # Check if it's IMU data (dict type)
        if isinstance(result, dict) and 'quaternion' in result:
            imu_data = result
            timestamp = time.time()

            # Update estimator (will initialize when enough samples)
            estimator.update_from_quaternion(
                imu_data['quaternion'],
                imu_data['linear_acceleration'],
                imu_data['angular_velocity'],
                timestamp
            )

            imu_count += 1
            if imu_count % 10 == 0:
                print(f"  Collected {imu_count} IMU samples...")

        # Timeout check
        if time.time() - init_start > 30:
            print("Initialization timeout. Make sure IMU data is being received.")
            lidar.disconnect()
            return

    print()
    print("Initialization complete!")
    print(f"  Gyro bias: {format_vector(estimator.gyro_bias)}")
    print(f"  Accel bias: {format_vector(estimator.accel_bias)}")
    print(f"  Gravity: {format_vector(estimator.gravity_world)}")
    print()

    print("=" * 60)
    print("  TRACKING PHASE")
    print("  Move the sensor around. Press Ctrl+C to stop.")
    print("=" * 60)
    print()

    # Tracking phase
    start_time = time.time()
    last_print_time = start_time
    update_count = 0
    max_distance = 0

    try:
        while True:
            packet = lidar.receive_packet()
            if packet is None:
                continue

            result = lidar.parse_packet(packet)

            # Process IMU data
            if isinstance(result, dict) and 'quaternion' in result:
                imu_data = result
                timestamp = time.time()

                # Calculate position
                position = estimator.update_from_quaternion(
                    imu_data['quaternion'],
                    imu_data['linear_acceleration'],
                    imu_data['angular_velocity'],
                    timestamp
                )

                update_count += 1
                distance = np.linalg.norm(position)
                max_distance = max(max_distance, distance)

                # Print status every 0.2 seconds
                if time.time() - last_print_time > 0.2:
                    state = estimator.get_state()
                    euler = state['euler_angles']
                    euler_deg = np.degrees(euler)

                    print(f"\r  Position: {format_vector(position, 3)} m  |  "
                          f"Velocity: {format_vector(state['velocity'], 2)} m/s  |  "
                          f"RPY: [{euler_deg[0]:+6.1f}, {euler_deg[1]:+6.1f}, {euler_deg[2]:+6.1f}]°  |  "
                          f"Distance: {distance:.3f} m  ", end="")

                    last_print_time = time.time()

    except KeyboardInterrupt:
        print("\n")
        print("=" * 60)
        print("  RESULTS")
        print("=" * 60)

        elapsed = time.time() - start_time
        final_state = estimator.get_state()

        print(f"  Duration: {elapsed:.1f} seconds")
        print(f"  IMU updates: {update_count}")
        print(f"  Update rate: {update_count/elapsed:.1f} Hz")
        print()
        print(f"  Final position: {format_vector(final_state['position'])} m")
        print(f"  Final velocity: {format_vector(final_state['velocity'])} m/s")
        print(f"  Max distance from start: {max_distance:.3f} m")
        print()

        # Show orientation
        euler = final_state['euler_angles']
        euler_deg = np.degrees(euler)
        print(f"  Final orientation (Euler):")
        print(f"    Roll:  {euler_deg[0]:+.2f}°")
        print(f"    Pitch: {euler_deg[1]:+.2f}°")
        print(f"    Yaw:   {euler_deg[2]:+.2f}°")
        print()

    finally:
        lidar.disconnect()
        print("Disconnected from LiDAR.")


def run_fusion_demo():
    """
    Demo combining IMU position with LiDAR odometry.

    This shows how to fuse high-frequency IMU updates with
    lower-frequency LiDAR scan matching.
    """
    print("=" * 60)
    print("  IMU-LiDAR Fusion Demo - Unitree L2")
    print("=" * 60)
    print()

    # Initialize LiDAR
    config = UnitreeL2Config()
    lidar = UnitreeL2UDP(config)

    print("Connecting to Unitree L2 LiDAR...")
    if not lidar.connect():
        print("Failed to connect.")
        return

    print("Connected!")

    # Initialize fusion system
    fusion = IMULiDARFusion()

    # Import SLAM for LiDAR odometry
    try:
        from src.data_processing.complete_slam import CompleteSLAM, SLAMConfig
        slam_config = SLAMConfig(
            voxel_size=0.05,
            max_range=15.0,
            loop_closure_enabled=False
        )
        slam = CompleteSLAM(slam_config)
        has_slam = True
        print("SLAM initialized for LiDAR odometry.")
    except ImportError:
        has_slam = False
        print("SLAM not available, using IMU-only mode.")

    print()
    print("Keep sensor STATIONARY for initialization...")

    # Initialize
    init_count = 0
    while not fusion.imu_estimator.initialized:
        packet = lidar.receive_packet()
        if packet is None:
            continue

        result = lidar.parse_packet(packet)
        if isinstance(result, dict) and 'quaternion' in result:
            fusion.imu_estimator.initialize_from_stationary(
                result['linear_acceleration'],
                result['angular_velocity']
            )
            init_count += 1
            if init_count % 10 == 0:
                print(f"  Initialization: {init_count}/50 samples...")

    print("Initialized! Move the sensor. Press Ctrl+C to stop.")
    print()

    # Tracking with fusion
    accumulated_points = []
    frames_per_scan = 3
    scan_count = 0

    try:
        while True:
            packet = lidar.receive_packet()
            if packet is None:
                continue

            result = lidar.parse_packet(packet)
            timestamp = time.time()

            if isinstance(result, dict) and 'quaternion' in result:
                # IMU update (high frequency)
                imu_data = result
                fused_pos = fusion.update_imu(
                    imu_data['quaternion'],
                    imu_data['linear_acceleration'],
                    imu_data['angular_velocity'],
                    timestamp
                )
                print(f"\r  [IMU] Position: {format_vector(fused_pos, 3)} m  ", end="")

            elif isinstance(result, tuple) and len(result) == 2:
                # Point cloud data
                points, intensities = result
                if len(points) > 50:
                    accumulated_points.append(points)

                    if len(accumulated_points) >= frames_per_scan:
                        # Process accumulated scan with SLAM
                        combined = np.vstack(accumulated_points)
                        accumulated_points = []

                        if has_slam:
                            transformed, pose, info = slam.process_scan(combined)
                            lidar_position = pose[:3, 3]

                            # LiDAR correction (low frequency)
                            fused_pos = fusion.update_lidar(lidar_position, timestamp)
                            scan_count += 1

                            print(f"\r  [LIDAR #{scan_count}] Fused: {format_vector(fused_pos, 3)} m  "
                                  f"| Map: {info.get('map_points', 0)} pts  ", end="")

    except KeyboardInterrupt:
        print("\n\nFusion demo complete.")
        state = fusion.get_state()
        print(f"Final fused position: {format_vector(state['fused_position'])} m")

    finally:
        lidar.disconnect()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="IMU Position Estimation Demo")
    parser.add_argument("--fusion", action="store_true",
                       help="Run IMU-LiDAR fusion demo instead of IMU-only")
    args = parser.parse_args()

    if args.fusion:
        run_fusion_demo()
    else:
        run_imu_position_demo()
