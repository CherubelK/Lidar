"""
Handheld SLAM Scanning - Optimized for motion

This script is optimized for handheld/moving LiDAR scanning:
- Higher processing frequency
- More tolerant motion thresholds
- Accumulates multiple frames before registration
- Better suited for walking around while scanning

Usage:
    python handheld_slam_scan.py [--duration SECONDS] [--name NAME]
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
    level=logging.WARNING,  # Reduce log spam for handheld
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def run_handheld_scan(duration: int = 60, name: str = "handheld_scan",
                      output_dir: str = "data/handheld_slam"):
    """
    Run handheld SLAM scan optimized for motion
    """
    print("\n" + "="*60)
    print("  HANDHELD SLAM SCAN")
    print("="*60)
    print(f"""
Optimized for handheld/moving scanning:
  - Accumulates points across frames
  - Higher motion tolerance
  - Faster processing

Tips:
  - Move smoothly (avoid jerky movements)
  - Scan overlapping areas
  - Walk slowly around the space
  - Return to start area for loop closure

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

    # Initialize SLAM with handheld-optimized settings
    print("\nInitializing SLAM (handheld mode)...")
    slam_config = SLAMConfig(
        voxel_size=0.1,            # Larger voxels = more tolerant of motion
        max_range=12.0,            # Shorter range for indoor
        map_voxel_size=0.05,       # Map resolution
        local_map_radius=20.0,     # Smaller local map
        loop_closure_enabled=True,
        loop_min_gap=30,           # Fewer scans needed for loop
        loop_dist_threshold=0.3,   # More tolerant matching
        save_trajectory=True,
        save_map=True
    )

    slam = CompleteSLAM(slam_config)
    print("[OK] SLAM initialized (handheld mode)")

    # Prepare output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    scan_name = f"{name}_{timestamp}"
    output_path = Path(output_dir) / scan_name
    output_path.mkdir(parents=True, exist_ok=True)

    print(f"\nOutput: {output_path}")
    print("\n" + "-"*60)
    print("Scanning... Move around the space. Press Ctrl+C to stop.")
    print("-"*60 + "\n")

    # Scanning loop - accumulate frames
    start_time = time.time()
    scan_count = 0
    frame_count = 0
    accumulated_points = []
    frames_per_scan = 3  # Accumulate 3 frames before processing
    loop_closures = []

    try:
        while time.time() - start_time < duration:
            # Get point cloud
            result = lidar.get_point_cloud_with_intensity()

            if result is None:
                continue

            if isinstance(result, dict):  # IMU data - skip for now
                continue

            points, intensities = result

            if len(points) < 50:
                continue

            frame_count += 1
            accumulated_points.append(points)

            # Process every N frames
            if len(accumulated_points) >= frames_per_scan:
                # Combine accumulated points
                combined_points = np.vstack(accumulated_points)
                accumulated_points = []

                # Downsample combined points
                if len(combined_points) > 10000:
                    indices = np.random.choice(len(combined_points), 10000, replace=False)
                    combined_points = combined_points[indices]

                # Process with SLAM
                transformed_points, pose, info = slam.process_scan(combined_points)

                scan_count += 1
                current_position = pose[:3, 3]

                # Check for loop closure
                if info['loop_detected']:
                    loop_closures.append({
                        'scan_idx': scan_count,
                        'loop_idx': info['loop_idx'],
                        'time': time.time() - start_time
                    })
                    print(f"\n>>> LOOP CLOSURE! Scan {scan_count} matched {info['loop_idx']}")
                    if info['optimized']:
                        print("    [OK] Poses optimized")

                # Progress update
                if scan_count % 5 == 0:
                    elapsed = int(time.time() - start_time)
                    stats = slam.get_stats()
                    print(f"[{elapsed:3d}s] Scan {scan_count:3d} | "
                          f"Frames: {frame_count:4d} | "
                          f"Map: {stats['map_points']:6d} pts | "
                          f"Pos: ({current_position[0]:5.1f}, {current_position[1]:5.1f}, {current_position[2]:5.1f})")

            time.sleep(0.02)  # ~50 Hz frame capture

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
Frames:         {frame_count}
Processed:      {final_stats['total_scans']} scans
Map points:     {final_stats['map_points']:,}
Loop closures:  {len(loop_closures)}
""")

    # Save results
    print(f"Saving results to {output_path}...")
    slam.save(str(output_path), prefix=scan_name)

    # Export for web viewer
    print("Exporting for web viewer...")
    export_for_web(slam, scan_name)

    print(f"\n[OK] Scan complete! View at http://localhost:8000/viewer.html")


def export_for_web(slam, scan_name):
    """Export scan to web viewer format"""
    try:
        web_models = Path("web/models")
        web_models.mkdir(parents=True, exist_ok=True)

        map_points = slam.get_map_points()
        if len(map_points) == 0:
            print("No points to export")
            return

        # Center points
        center = map_points.mean(axis=0)
        points_centered = map_points - center

        # Flatten for web viewer
        vertices_flat = points_centered.flatten().tolist()

        model_data = {
            'vertices': vertices_flat,
            'faces': []
        }

        output_file = web_models / f"{scan_name}.json"
        with open(output_file, 'w') as f:
            json.dump(model_data, f)

        print(f"[OK] Web model: {output_file}")

    except Exception as e:
        print(f"Warning: Could not export web model: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Handheld SLAM scanning optimized for motion",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--duration', type=int, default=60,
                       help='Scan duration in seconds (default: 60)')
    parser.add_argument('--name', type=str, default='handheld_scan',
                       help='Name for this scan')
    parser.add_argument('--output-dir', type=str, default='data/handheld_slam',
                       help='Output directory')

    args = parser.parse_args()

    run_handheld_scan(
        duration=args.duration,
        name=args.name,
        output_dir=args.output_dir
    )


if __name__ == "__main__":
    main()