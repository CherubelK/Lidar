"""
Complete SLAM Scanning with Loop Closure

This example demonstrates the full SLAM pipeline:
1. KISS-ICP odometry for frame-to-frame motion estimation
2. ikd-Tree for efficient map storage and queries
3. Scan Context for loop closure detection
4. Pose Graph Optimization for drift correction

Usage:
    python complete_slam_scan.py [--duration SECONDS] [--name NAME]

For best results:
- Move SLOWLY (10-20cm per step)
- Return to starting location to trigger loop closure
- The system will automatically detect the loop and correct drift
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
from src.data_processing.complete_slam import CompleteSLAM, SLAMConfig

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def run_slam_scan(duration: int = 120, name: str = "slam_scan",
                 output_dir: str = "data/complete_slam"):
    """
    Run complete SLAM scan with loop closure

    Args:
        duration: Scan duration in seconds
        name: Name for this scan
        output_dir: Output directory
    """
    print("\n" + "="*60)
    print("  COMPLETE SLAM WITH LOOP CLOSURE")
    print("="*60)
    print(f"""
Components:
  - Odometry: KISS-ICP (frame-to-frame matching)
  - Map: ikd-Tree (incremental k-d tree)
  - Loop Closure: Scan Context (place recognition)
  - Optimization: Pose Graph (drift correction)

Instructions:
  1. Move SLOWLY (10-20cm steps, wait 2-3 seconds)
  2. Walk in a loop pattern (room perimeter)
  3. Return to starting position
  4. System will detect loop and correct drift

Duration: {duration} seconds
""")

    # Initialize LiDAR
    print("Initializing LiDAR...")
    config = UnitreeL2Config()
    lidar = UnitreeL2UDP(config)

    if not lidar.connect():
        print("ERROR: Failed to connect to LiDAR")
        print("\nTroubleshooting:")
        print("  1. Check LiDAR power and network connection")
        print("  2. Verify IP: ping 192.168.1.62")
        print("  3. Check your network adapter is 192.168.1.2")
        return

    print("[OK] LiDAR connected")

    # Initialize SLAM system
    print("\nInitializing SLAM system...")
    slam_config = SLAMConfig(
        voxel_size=0.05,           # 5cm voxels for odometry
        max_range=15.0,            # 15m max range (indoor)
        map_voxel_size=0.02,       # 2cm voxels for map
        local_map_radius=30.0,     # 30m local map
        loop_closure_enabled=True,
        loop_min_gap=50,           # Min 50 scans between loop candidates
        loop_dist_threshold=0.25,  # Scan Context threshold
        save_trajectory=True,
        save_map=True
    )

    slam = CompleteSLAM(slam_config)
    print("[OK] SLAM system initialized")

    # Prepare output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    scan_name = f"{name}_{timestamp}"
    output_path = Path(output_dir) / scan_name
    output_path.mkdir(parents=True, exist_ok=True)

    print(f"\nOutput: {output_path}")
    print("\n" + "-"*60)
    print("Starting scan... (Press Ctrl+C to stop early)")
    print("-"*60 + "\n")

    # Scanning loop
    start_time = time.time()
    scan_count = 0
    loop_closures = []
    last_position = np.zeros(3)
    total_distance = 0

    try:
        while time.time() - start_time < duration:
            # Get point cloud
            result = lidar.get_point_cloud_with_intensity()

            if result is None:
                continue

            if isinstance(result, dict):  # IMU data
                continue

            points, intensities = result

            if len(points) < 100:
                continue

            # Process with SLAM
            transformed_points, pose, info = slam.process_scan(points)

            scan_count += 1
            current_position = pose[:3, 3]

            # Calculate movement
            movement = np.linalg.norm(current_position - last_position)
            if scan_count > 1:
                total_distance += movement
            last_position = current_position.copy()

            # Check for loop closure
            if info['loop_detected']:
                loop_closures.append({
                    'scan_idx': scan_count,
                    'loop_idx': info['loop_idx'],
                    'time': time.time() - start_time
                })
                print(f"\n{'='*50}")
                print(f">>> LOOP CLOSURE DETECTED!")
                print(f"   Current scan: {scan_count}")
                print(f"   Matched with: {info['loop_idx']}")
                if info['optimized']:
                    print(f"   [OK] Pose graph optimized!")
                print(f"{'='*50}\n")

            # Progress update every 10 scans
            if scan_count % 10 == 0:
                elapsed = int(time.time() - start_time)
                remaining = duration - elapsed
                stats = slam.get_stats()

                print(f"[{elapsed:3d}s] Scan {scan_count:4d} | "
                      f"Pos: ({current_position[0]:6.2f}, {current_position[1]:6.2f}, {current_position[2]:6.2f}) | "
                      f"Map: {stats['map_points']:6d} pts | "
                      f"Loops: {len(loop_closures)} | "
                      f"Dist: {total_distance:.1f}m")

            # Movement warning
            if movement > 0.5 and scan_count > 1:
                print(f"  WARNING: Moving too fast! ({movement:.2f}m) - slow down for better tracking")

            time.sleep(0.1)  # ~10 Hz

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
""")

    # Check loop closure quality
    trajectory = slam.get_trajectory()
    if len(trajectory) > 1:
        start_pos = trajectory[0]
        end_pos = trajectory[-1]
        gap = np.linalg.norm(end_pos - start_pos)

        print(f"Start position: ({start_pos[0]:.2f}, {start_pos[1]:.2f}, {start_pos[2]:.2f})")
        print(f"End position:   ({end_pos[0]:.2f}, {end_pos[1]:.2f}, {end_pos[2]:.2f})")
        print(f"Loop gap:       {gap:.3f}m")

        if len(loop_closures) > 0:
            if gap < 0.5:
                print("\n[OK] Excellent loop closure! Drift well corrected.")
            elif gap < 1.0:
                print("\n[OK] Good loop closure. Some residual drift.")
            else:
                print("\n[NOTE] Large gap - try walking a tighter loop or moving slower.")
        else:
            print("\n[NOTE] No loop closures detected.")
            print("   Tip: Return to your starting position to trigger loop closure.")

    # Save results
    print(f"\nSaving results to {output_path}...")
    slam.save(str(output_path), prefix=scan_name)

    # Save loop closure info
    loop_info = {
        'scan_name': scan_name,
        'duration': elapsed_time,
        'total_scans': final_stats['total_scans'],
        'map_points': final_stats['map_points'],
        'path_length': final_stats['trajectory_length'],
        'loop_closures': loop_closures,
        'final_gap': float(gap) if len(trajectory) > 1 else None
    }

    with open(output_path / f"{scan_name}_loop_info.json", 'w') as f:
        json.dump(loop_info, f, indent=2)

    # Generate web viewer compatible output
    try:
        print("\nGenerating web viewer model...")
        map_points = slam.get_map_points()

        if len(map_points) > 0:
            # Center points for better viewing
            center = map_points.mean(axis=0)
            points_centered = map_points - center

            # Create web viewer format
            model_data = {
                'vertices': points_centered.flatten().tolist(),
                'faces': []  # Point cloud mode - no faces
            }

            web_output = Path("web/models") / f"{scan_name}.json"
            web_output.parent.mkdir(parents=True, exist_ok=True)

            with open(web_output, 'w') as f:
                json.dump(model_data, f)

            print(f"[OK] Web model saved: {web_output}")
            print(f"  View at: http://localhost:8000/viewer.html")

    except Exception as e:
        logger.warning(f"Could not generate web model: {e}")

    print(f"\n[OK] All results saved to: {output_path}")
    print("\nFiles created:")
    for f in sorted(output_path.glob("*")):
        size = f.stat().st_size
        if size > 1024*1024:
            size_str = f"{size/1024/1024:.1f} MB"
        elif size > 1024:
            size_str = f"{size/1024:.1f} KB"
        else:
            size_str = f"{size} B"
        print(f"  {f.name}: {size_str}")


def main():
    parser = argparse.ArgumentParser(
        description="Complete SLAM scanning with loop closure",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python complete_slam_scan.py
  python complete_slam_scan.py --duration 180 --name office_loop
  python complete_slam_scan.py --duration 300 --name warehouse

Tips for best results:
  - Move SLOWLY (10-20cm steps)
  - Wait 2-3 seconds between steps
  - Walk in a loop (return to start)
  - Scan for at least 60 seconds
  - Ensure 60-70% overlap between scans
        """
    )

    parser.add_argument('--duration', type=int, default=120,
                       help='Scan duration in seconds (default: 120)')

    parser.add_argument('--name', type=str, default='slam_scan',
                       help='Name for this scan (default: slam_scan)')

    parser.add_argument('--output-dir', type=str, default='data/complete_slam',
                       help='Output directory (default: data/complete_slam)')

    args = parser.parse_args()

    run_slam_scan(
        duration=args.duration,
        name=args.name,
        output_dir=args.output_dir
    )


if __name__ == "__main__":
    main()