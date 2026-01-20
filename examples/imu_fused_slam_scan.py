"""
IMU-Fused SLAM Scan - Full Sensor Data Utilization

This script properly uses ALL data from the Unitree L2 LiDAR:
1. Point cloud data (3D or 2D packets)
2. IMU quaternion orientation
3. IMU angular velocity (gyroscope)
4. IMU linear acceleration (accelerometer)

The IMU data is used to:
- Transform each point cloud frame to a gravity-aligned world frame
- Track position through double integration of acceleration
- Provide motion priors for better odometry

Usage:
    python examples/imu_fused_slam_scan.py [--duration SECONDS] [--name NAME]
"""

import sys
from pathlib import Path
import argparse
import time
import numpy as np
from datetime import datetime
import json
import logging

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lidar_interface.unitree_l2_udp import UnitreeL2UDP, UnitreeL2Config
from src.data_processing.imu_fused_slam import IMUFusedSLAM, IMUFusedSLAMConfig

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def run_imu_fused_scan(duration: int = 60, name: str = "imu_fused_scan",
                       output_dir: str = "data/imu_fused_slam",
                       imu_weight: float = 0.3):
    """
    Run IMU-fused SLAM scan.

    Args:
        duration: Scan duration in seconds
        name: Name for this scan
        output_dir: Output directory
        imu_weight: Weight for IMU vs odometry (0=odometry only, 1=IMU only)
    """
    print("\n" + "="*60)
    print("  IMU-FUSED SLAM - FULL SENSOR DATA UTILIZATION")
    print("="*60)
    print(f"""
Sensor Data Used:
  - Point Cloud: 3D points from LiDAR scanner
  - Quaternion: IMU orientation (gravity-aligned world frame)
  - Gyroscope: Angular velocity for motion tracking
  - Accelerometer: Linear acceleration for position estimation

IMU Fusion Settings:
  - IMU Weight: {imu_weight:.0%} IMU / {1-imu_weight:.0%} Odometry
  - Uses IMU quaternion for frame alignment
  - Gravity-compensated acceleration for position

Duration: {duration} seconds
""")

    # Initialize LiDAR
    print("Initializing LiDAR...")
    config = UnitreeL2Config()
    lidar = UnitreeL2UDP(config)

    if not lidar.connect():
        print("ERROR: Failed to connect to LiDAR")
        return

    print("[OK] LiDAR connected")

    # Initialize IMU-fused SLAM
    print("\nInitializing IMU-Fused SLAM system...")
    slam_config = IMUFusedSLAMConfig(
        voxel_size=0.05,
        max_range=15.0,
        map_voxel_size=0.02,
        use_imu_orientation=True,
        use_imu_position=True,
        imu_weight=imu_weight,
        loop_closure_enabled=True,
        loop_min_gap=30,
        loop_dist_threshold=0.25
    )

    slam = IMUFusedSLAM(slam_config)
    print("[OK] IMU-Fused SLAM initialized")

    # Prepare output
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    scan_name = f"{name}_{timestamp}"
    output_path = Path(output_dir) / scan_name
    output_path.mkdir(parents=True, exist_ok=True)

    print(f"\nOutput: {output_path}")

    # IMU initialization phase
    print("\n" + "-"*60)
    print("PHASE 1: IMU INITIALIZATION")
    print("Keep the sensor STATIONARY for calibration...")
    print("-"*60)

    init_start = time.time()
    imu_count = 0
    point_count = 0

    while not slam.imu_state.initialized:
        packet = lidar.receive_packet()
        if packet is None:
            continue

        result = lidar.parse_packet(packet)

        if isinstance(result, dict) and 'quaternion' in result:
            # IMU data
            imu_data = result
            slam.update_imu(
                imu_data['quaternion'],
                imu_data['linear_acceleration'],
                imu_data['angular_velocity'],
                time.time()
            )
            imu_count += 1
            if imu_count % 10 == 0:
                print(f"  IMU samples: {imu_count}/30...")

        elif isinstance(result, tuple):
            point_count += 1

        # Timeout
        if time.time() - init_start > 30:
            print("WARNING: IMU initialization timeout - proceeding anyway")
            break

    if slam.imu_state.initialized:
        print("\n[OK] IMU initialized successfully!")
        print(f"  Gravity vector: [{slam.imu_state.gravity_world[0]:.3f}, "
              f"{slam.imu_state.gravity_world[1]:.3f}, "
              f"{slam.imu_state.gravity_world[2]:.3f}]")
    else:
        print("\n[NOTE] IMU not initialized - using odometry only")

    # Scanning phase
    print("\n" + "-"*60)
    print("PHASE 2: SCANNING")
    print("Move the sensor slowly. Press Ctrl+C to stop.")
    print("-"*60 + "\n")

    start_time = time.time()
    scan_count = 0
    imu_updates = 0
    loop_closures = []
    last_print_time = start_time
    accumulated_points = []
    frames_per_scan = 3  # Accumulate frames for denser scans

    try:
        while time.time() - start_time < duration:
            packet = lidar.receive_packet()
            if packet is None:
                continue

            result = lidar.parse_packet(packet)
            current_time = time.time()

            if isinstance(result, dict) and 'quaternion' in result:
                # IMU data - update at high frequency
                imu_data = result
                slam.update_imu(
                    imu_data['quaternion'],
                    imu_data['linear_acceleration'],
                    imu_data['angular_velocity'],
                    current_time
                )
                imu_updates += 1

            elif isinstance(result, tuple) and len(result) == 2:
                # Point cloud data
                points, intensities = result

                if len(points) < 50:
                    continue

                accumulated_points.append(points)

                # Process accumulated frames
                if len(accumulated_points) >= frames_per_scan:
                    combined_points = np.vstack(accumulated_points)
                    accumulated_points = []

                    # Process with IMU-fused SLAM
                    transformed, pose, info = slam.process_scan(
                        combined_points, current_time
                    )

                    scan_count += 1
                    position = pose[:3, 3]

                    # Check for loop closure
                    if info['loop_detected']:
                        loop_closures.append({
                            'scan_idx': scan_count,
                            'loop_idx': info['loop_idx'],
                            'time': current_time - start_time
                        })
                        print(f"\n>>> LOOP CLOSURE: {scan_count} -> {info['loop_idx']}")
                        if info['optimized']:
                            print("    Pose graph optimized!")

                    # Print status every 0.5 seconds
                    if current_time - last_print_time > 0.5:
                        elapsed = int(current_time - start_time)
                        stats = slam.get_stats()

                        print(f"[{elapsed:3d}s] Scan {scan_count:4d} | "
                              f"Pos: ({position[0]:6.2f}, {position[1]:6.2f}, {position[2]:6.2f}) | "
                              f"Map: {stats['map_points']:6d} pts | "
                              f"IMU: {imu_updates:5d} | "
                              f"Loops: {len(loop_closures)}")

                        last_print_time = current_time

            # Small delay to prevent CPU overload
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nScan stopped by user")

    finally:
        lidar.disconnect()

    # Final statistics
    elapsed_time = time.time() - start_time
    final_stats = slam.get_stats()

    print("\n" + "="*60)
    print("  SCAN COMPLETE")
    print("="*60)
    print(f"""
Duration:       {elapsed_time:.1f} seconds
Total scans:    {final_stats['total_scans']}
Map points:     {final_stats['map_points']:,}
Path length:    {final_stats['trajectory_length']:.2f}m
Loop closures:  {len(loop_closures)}
IMU updates:    {final_stats['imu_updates']:,}
IMU rate:       {final_stats['imu_updates']/elapsed_time:.0f} Hz
""")

    # Save results
    print(f"Saving results to {output_path}...")
    slam.save(str(output_path), prefix=scan_name)

    # Save scan info
    scan_info = {
        'scan_name': scan_name,
        'duration': elapsed_time,
        'total_scans': final_stats['total_scans'],
        'map_points': final_stats['map_points'],
        'path_length': final_stats['trajectory_length'],
        'loop_closures': loop_closures,
        'imu_updates': final_stats['imu_updates'],
        'imu_initialized': final_stats['imu_initialized'],
        'config': {
            'imu_weight': imu_weight,
            'use_imu_orientation': slam_config.use_imu_orientation,
            'use_imu_position': slam_config.use_imu_position
        }
    }

    with open(output_path / f"{scan_name}_info.json", 'w') as f:
        json.dump(scan_info, f, indent=2)

    # Export for web viewer
    export_for_web(slam, scan_name)

    print(f"\n[OK] All results saved to: {output_path}")


def export_for_web(slam, scan_name):
    """Export scan to web viewer format."""
    try:
        web_models = Path("web/models")
        web_models.mkdir(parents=True, exist_ok=True)

        map_points = slam.get_map_points()
        if len(map_points) == 0:
            print("No points to export")
            return

        # Center points for viewing
        center = map_points.mean(axis=0)
        points_centered = map_points - center

        # Create web format
        model_data = {
            'vertices': points_centered.flatten().tolist(),
            'faces': []
        }

        output_file = web_models / f"{scan_name}.json"
        with open(output_file, 'w') as f:
            json.dump(model_data, f)

        print(f"[OK] Web model: {output_file}")
        print(f"  View at: http://localhost:8000/viewer.html")

        # Update index
        update_model_index(scan_name, str(output_file))

    except Exception as e:
        print(f"Warning: Could not export web model: {e}")


def update_model_index(scan_name: str, filepath: str):
    """Update the web models index.json."""
    try:
        index_file = Path("web/models/index.json")
        if index_file.exists():
            with open(index_file, 'r') as f:
                index = json.load(f)
        else:
            index = {'scans': []}

        # Add new scan at the beginning
        new_entry = {
            'name': scan_name,
            'file': Path(filepath).name
        }

        # Check if already exists
        existing_names = [s['name'] for s in index['scans']]
        if scan_name not in existing_names:
            index['scans'].insert(0, new_entry)

            with open(index_file, 'w') as f:
                json.dump(index, f, indent=2)

    except Exception as e:
        logger.warning(f"Could not update index: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="IMU-Fused SLAM scanning with full sensor data",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python imu_fused_slam_scan.py
  python imu_fused_slam_scan.py --duration 120 --name office
  python imu_fused_slam_scan.py --imu-weight 0.5  # More IMU influence

IMU Weight Settings:
  0.0 = Odometry only (ignores IMU position)
  0.3 = Default (balanced fusion)
  0.5 = Equal weight
  1.0 = IMU only (may drift over time)
        """
    )

    parser.add_argument('--duration', type=int, default=60,
                       help='Scan duration in seconds (default: 60)')
    parser.add_argument('--name', type=str, default='imu_fused_scan',
                       help='Name for this scan')
    parser.add_argument('--output-dir', type=str, default='data/imu_fused_slam',
                       help='Output directory')
    parser.add_argument('--imu-weight', type=float, default=0.3,
                       help='IMU weight for fusion (0=odometry, 1=IMU only)')

    args = parser.parse_args()

    run_imu_fused_scan(
        duration=args.duration,
        name=args.name,
        output_dir=args.output_dir,
        imu_weight=args.imu_weight
    )


if __name__ == "__main__":
    main()
