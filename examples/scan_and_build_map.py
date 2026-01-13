"""
Complete Trail Scanning and 3D Map Building Pipeline

This script demonstrates the complete workflow:
1. Capture LiDAR data from trail
2. Process and clean point cloud
3. Generate 3D mesh
4. Export for web visualization
"""

import sys
from pathlib import Path
import numpy as np
import logging
import argparse
from datetime import datetime

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lidar_interface import LiDARDataCapture, LiDARConfig
from src.data_processing.point_cloud_processor_numpy import PointCloudProcessorNumPy
from src.data_processing.mesh_generator import MeshGenerator

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def scan_trail(trail_name: str, duration: int = 60, use_real_sensor: bool = False):
    """
    Scan a trail and capture LiDAR data.

    Args:
        trail_name: Name for this trail
        duration: How long to scan in seconds
        use_real_sensor: If True, use real Unitree L2 sensor; if False, use synthetic data

    Returns:
        List of captured point cloud frames
    """
    logger.info(f"Starting trail scan: {trail_name}")
    logger.info(f"Duration: {duration} seconds")

    if use_real_sensor:
        # Configure for real Unitree L2 sensor
        config = LiDARConfig(
            ip_address="192.168.1.62",
            port=6101,
            host_ip="192.168.1.2",
            host_port=6201,
            use_udp=True
        )

        capture = LiDARDataCapture(output_dir="data/raw", lidar_config=config)

        # Start capture session
        if not capture.start_session(trail_name):
            logger.error("Failed to start capture session!")
            return None

        # Capture data
        logger.info(f"Capturing data for {duration} seconds...")
        frames_captured = capture.capture_continuous(duration_seconds=duration)

        # End session
        capture.end_session()

        if frames_captured == 0:
            logger.error("No frames captured!")
            return None

        # Load captured frames
        logger.info(f"Loading {frames_captured} captured frames...")
        frames = capture.load_session_frames(trail_name)
        return frames

    else:
        # Use synthetic data for testing
        logger.info("Using synthetic test data (set use_real_sensor=True for real hardware)")

        frames = []
        num_frames = max(1, duration // 10)  # ~10 frames per second simulated

        for i in range(num_frames):
            # Generate synthetic trail segment
            num_points = 1000
            x = np.random.uniform(-5, 5, num_points)
            y = np.random.uniform(i * 2, (i + 1) * 2, num_points)  # Progress along trail
            z = np.sin(y * 0.3) * 0.5 + np.random.normal(0, 0.05, num_points)

            # Add some features
            features_x = np.random.uniform(-3, 3, num_points // 2)
            features_y = np.random.uniform(i * 2, (i + 1) * 2, num_points // 2)
            features_z = np.random.uniform(0.2, 2.0, num_points // 2)

            ground = np.column_stack([x, y, z])
            features = np.column_stack([features_x, features_y, features_z])
            frame = np.vstack([ground, features])

            frames.append(frame)

        logger.info(f"Generated {len(frames)} synthetic frames")
        return frames


def process_trail_data(frames: list, trail_name: str):
    """
    Process captured trail data into a clean point cloud.

    Args:
        frames: List of point cloud frames
        trail_name: Trail name for saving

    Returns:
        Processed point cloud data dictionary
    """
    logger.info("Processing trail data...")

    # Merge all frames
    processor = PointCloudProcessorNumPy()
    merged_points = processor.merge_point_clouds(frames)
    logger.info(f"Merged {len(frames)} frames into {len(merged_points)} total points")

    # Process the complete point cloud
    result = processor.process_trail_scan(
        merged_points,
        remove_outliers=True,
        downsample_voxel=0.05,
        segment_ground=True
    )

    # Save processed data
    output_path = f"data/processed/{trail_name}_processed.npz"
    processor.save_processed_cloud(
        result['processed_points'],
        result['normals'],
        output_path
    )

    logger.info(f"Processed data saved to: {output_path}")
    logger.info(f"Final point count: {result['num_final_points']}")

    return result


def build_3d_mesh(processed_data: dict, trail_name: str):
    """
    Build 3D mesh from processed point cloud.

    Args:
        processed_data: Processed point cloud dictionary
        trail_name: Trail name for saving

    Returns:
        Mesh data dictionary
    """
    logger.info("Building 3D mesh...")

    generator = MeshGenerator()

    # Create mesh
    mesh_data = generator.create_trail_mesh(
        processed_data['processed_points'],
        processed_data['normals'],
        method='delaunay'
    )

    logger.info(f"Mesh created: {mesh_data['num_vertices']} vertices, {mesh_data['num_faces']} faces")

    return mesh_data


def export_for_web(mesh_data: dict, trail_name: str):
    """
    Export mesh in formats suitable for web visualization.

    Args:
        mesh_data: Mesh data dictionary
        trail_name: Trail name for saving

    Returns:
        Dictionary of exported file paths
    """
    logger.info("Exporting for web visualization...")

    generator = MeshGenerator()

    # Export all formats
    base_path = f"web/models/{trail_name}"
    exports = generator.export_all_formats(mesh_data, base_path)

    logger.info("Exported files:")
    for format_name, file_path in exports.items():
        logger.info(f"  {format_name.upper()}: {file_path}")

    return exports


def main():
    """Main pipeline execution."""
    parser = argparse.ArgumentParser(description='Scan trail and build 3D map')
    parser.add_argument('trail_name', type=str, help='Name for this trail')
    parser.add_argument('--duration', type=int, default=60, help='Scan duration in seconds')
    parser.add_argument('--real-sensor', action='store_true', help='Use real Unitree L2 sensor')
    parser.add_argument('--skip-scan', action='store_true', help='Skip scanning, load existing data')

    args = parser.parse_args()

    print("=" * 60)
    print("Trail Scanning and 3D Map Building Pipeline")
    print("=" * 60)
    print(f"Trail Name: {args.trail_name}")
    print(f"Mode: {'Real Sensor' if args.real_sensor else 'Synthetic Data'}")
    print("=" * 60)
    print()

    try:
        # Step 1: Scan trail (or load existing)
        if args.skip_scan:
            logger.info("Loading existing scan data...")
            capture = LiDARDataCapture(output_dir="data/raw")
            frames = capture.load_session_frames(args.trail_name)

            if not frames:
                logger.error(f"No existing scan found for '{args.trail_name}'")
                return

        else:
            frames = scan_trail(
                args.trail_name,
                duration=args.duration,
                use_real_sensor=args.real_sensor
            )

            if not frames:
                logger.error("Scanning failed!")
                return

        # Step 2: Process data
        processed_data = process_trail_data(frames, args.trail_name)

        # Step 3: Build mesh
        mesh_data = build_3d_mesh(processed_data, args.trail_name)

        # Step 4: Export for web
        exports = export_for_web(mesh_data, args.trail_name)

        # Summary
        print()
        print("=" * 60)
        print("Pipeline Complete!")
        print("=" * 60)
        print(f"Trail: {args.trail_name}")
        print(f"Total Points: {processed_data['num_raw_points']:,}")
        print(f"Processed Points: {processed_data['num_final_points']:,}")
        print(f"Mesh Vertices: {mesh_data['num_vertices']:,}")
        print(f"Mesh Faces: {mesh_data['num_faces']:,}")
        print()
        print("Exported Files:")
        for format_name, file_path in exports.items():
            print(f"  {format_name.upper()}: {file_path}")
        print()
        print("To view in 3D:")
        print(f"  1. Open web/index.html in a browser")
        print(f"  2. Select '{args.trail_name}' from the dropdown")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nOperation cancelled by user")

    except Exception as e:
        logger.error(f"Pipeline failed: {e}", exc_info=True)


if __name__ == "__main__":
    main()