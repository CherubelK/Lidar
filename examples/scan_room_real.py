"""
Real Unitree L2 LiDAR Room Scanning
Capture real data from the Unitree L2 sensor and build a 3D model.
"""

import sys
from pathlib import Path
import numpy as np
import logging
from datetime import datetime
import time
import argparse

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lidar_interface.unitree_l2_udp import UnitreeL2UDP, UnitreeL2Config
from src.data_processing.point_cloud_processor_numpy import PointCloudProcessorNumPy
from src.data_processing.mesh_generator import MeshGenerator

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def scan_room(duration_seconds=30, room_name="my_room"):
    """
    Scan a room with the Unitree L2 LiDAR sensor.

    Args:
        duration_seconds: How long to scan
        room_name: Name for the scan

    Returns:
        List of point cloud frames
    """
    print("=" * 70)
    print("UNITREE L2 REAL SENSOR ROOM SCANNING")
    print("=" * 70)
    print()
    print(f"Room: {room_name}")
    print(f"Duration: {duration_seconds} seconds")
    print()

    # Connect to sensor
    config = UnitreeL2Config()
    receiver = UnitreeL2UDP(config)

    print("Connecting to Unitree L2...")
    if not receiver.connect():
        print("ERROR: Failed to connect to sensor!")
        print()
        print("Troubleshooting:")
        print("  1. Check sensor is powered on")
        print("  2. Verify network: ping 192.168.1.62")
        print("  3. Check your PC IP is 192.168.1.2")
        return None

    print("Connected! Starting scan...")
    print()
    print(f"Scanning for {duration_seconds} seconds - move sensor around the room slowly...")
    print("Press Ctrl+C to stop early")
    print()

    frames = []
    start_time = time.time()
    packet_count = 0
    point_count = 0

    try:
        while True:
            elapsed = time.time() - start_time

            if elapsed >= duration_seconds:
                break

            # Get point cloud from sensor
            points = receiver.get_point_cloud()

            if points is not None and len(points) > 0:
                frames.append(points)
                packet_count += 1
                point_count += len(points)

                # Progress update every second
                if packet_count % 10 == 0:
                    print(f"  {elapsed:.1f}s - {packet_count} packets, {point_count:,} points total")

    except KeyboardInterrupt:
        print("\nScan stopped by user")

    finally:
        receiver.disconnect()

    print()
    print(f"Scan complete!")
    print(f"  Duration: {time.time() - start_time:.1f} seconds")
    print(f"  Packets: {packet_count}")
    print(f"  Total points: {point_count:,}")
    print(f"  Frames captured: {len(frames)}")
    print()

    return frames


def main():
    parser = argparse.ArgumentParser(description='Scan a room with Unitree L2 LiDAR')
    parser.add_argument('--room', default='my_room', help='Room name (default: my_room)')
    parser.add_argument('--duration', type=int, default=30, help='Scan duration in seconds (default: 30)')
    args = parser.parse_args()

    # Scan the room
    frames = scan_room(duration_seconds=args.duration, room_name=args.room)

    if not frames or len(frames) == 0:
        print("ERROR: No data captured!")
        return

    # Process the data
    print("Processing point cloud...")
    processor = PointCloudProcessorNumPy()

    merged = processor.merge_point_clouds(frames)
    print(f"  Merged: {len(merged):,} total points")

    result = processor.process_trail_scan(
        merged,
        remove_outliers=True,
        downsample_voxel=0.02,  # 2cm voxels for room detail
        segment_ground=True
    )

    print(f"  Processed: {result['num_final_points']:,} points")

    # Save processed data
    output_dir = Path("data/processed")
    output_dir.mkdir(parents=True, exist_ok=True)

    processor.save_processed_cloud(
        result['processed_points'],
        result['normals'],
        f"data/processed/{args.room}_processed.npz"
    )

    # Build 3D mesh
    print()
    print("Building 3D mesh...")
    generator = MeshGenerator()

    mesh_data = generator.create_trail_mesh(
        result['processed_points'],
        result['normals'],
        method='delaunay'
    )

    print(f"  Mesh: {mesh_data['num_vertices']:,} vertices, {mesh_data['num_faces']:,} faces")

    # Export for web
    print()
    print("Exporting for web visualization...")
    exports = generator.export_all_formats(mesh_data, f"web/models/{args.room}")

    # Summary
    print()
    print("=" * 70)
    print("ROOM SCAN COMPLETE!")
    print("=" * 70)
    print(f"Room: {args.room}")
    print(f"Raw points: {len(merged):,}")
    print(f"Processed points: {result['num_final_points']:,}")
    print(f"Mesh vertices: {mesh_data['num_vertices']:,}")
    print(f"Mesh faces: {mesh_data['num_faces']:,}")
    print()
    print("Files:")
    for fmt, path in exports.items():
        size_kb = Path(path).stat().st_size / 1024
        print(f"  {fmt.upper()}: {path} ({size_kb:.0f} KB)")
    print()
    print("To view in 3D:")
    print("  1. Make sure web server is running:")
    print("     cd web && python server.py")
    print("  2. Open http://localhost:8000")
    print(f"  3. Select '{args.room}' from dropdown")
    print()
    print("=" * 70)


if __name__ == "__main__":
    main()