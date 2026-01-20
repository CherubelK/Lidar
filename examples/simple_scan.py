"""
Simple LiDAR Scan - Minimal Working Example

This script captures point cloud data from the Unitree L2 LiDAR
using the simplest correct approach:

1. Receive point cloud packets (already in sensor Cartesian coordinates)
2. Use IMU quaternion to transform to world frame (gravity-aligned)
3. Accumulate points with simple voxel deduplication
4. Save for web viewer

No complex SLAM, no pose graph - just direct IMU-based transforms.
"""

import sys
from pathlib import Path
import time
import numpy as np
from datetime import datetime
import json

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lidar_interface.unitree_l2_udp import UnitreeL2UDP, UnitreeL2Config


def quaternion_to_rotation_matrix(q):
    """
    Convert quaternion [w, x, y, z] to 3x3 rotation matrix.

    This is the standard formula - no fancy libraries needed.
    """
    w, x, y, z = q

    # Normalize quaternion
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-10:
        return np.eye(3)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm

    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),       2*(x*z + w*y)],
        [2*(x*y + w*z),         1 - 2*(x*x + z*z),   2*(y*z - w*x)],
        [2*(x*z - w*y),         2*(y*z + w*x),       1 - 2*(x*x + y*y)]
    ])


def voxel_downsample(points, voxel_size=0.02):
    """
    Simple voxel grid downsampling - removes duplicate points.
    """
    if len(points) == 0:
        return points

    # Quantize to voxel grid
    voxel_indices = np.floor(points / voxel_size).astype(np.int32)

    # Get unique voxels
    _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)

    return points[unique_indices]


def statistical_outlier_removal(points, k=20, std_ratio=2.0):
    """
    Remove outlier points based on mean distance to k nearest neighbors.
    Points with distance > mean + std_ratio * std are removed.
    """
    if len(points) < k + 1:
        return points

    from scipy.spatial import cKDTree

    # Build KD-tree for fast neighbor queries
    tree = cKDTree(points)

    # Find k nearest neighbors for each point (excluding itself)
    distances, _ = tree.query(points, k=k+1)
    mean_distances = distances[:, 1:].mean(axis=1)  # Skip first (self)

    # Calculate threshold
    global_mean = mean_distances.mean()
    global_std = mean_distances.std()
    threshold = global_mean + std_ratio * global_std

    # Keep points below threshold
    mask = mean_distances < threshold
    return points[mask]


def remove_sparse_points(points, radius=0.1, min_neighbors=5):
    """
    Remove points that don't have enough neighbors within a radius.
    This removes isolated streaks/lines while keeping dense surfaces.
    """
    if len(points) < min_neighbors:
        return points

    from scipy.spatial import cKDTree

    tree = cKDTree(points)

    # Count neighbors within radius for each point
    neighbor_counts = tree.query_ball_point(points, r=radius, return_length=True)

    # Keep points with enough neighbors (subtract 1 for self)
    mask = np.array(neighbor_counts) >= min_neighbors
    return points[mask]


def remove_linear_structures(points, k=10, linearity_threshold=0.8):
    """
    Remove points that lie on linear structures (streaks/lines).
    Uses PCA to detect if local neighborhood is linear.
    """
    if len(points) < k + 1:
        return points

    from scipy.spatial import cKDTree

    tree = cKDTree(points)
    _, indices = tree.query(points, k=k+1)

    mask = np.ones(len(points), dtype=bool)

    for i, neighbor_idx in enumerate(indices):
        # Get local neighborhood
        neighbors = points[neighbor_idx[1:]]  # Exclude self

        # Center the points
        centered = neighbors - neighbors.mean(axis=0)

        # Compute covariance matrix
        cov = np.cov(centered.T)

        # Get eigenvalues
        eigenvalues = np.linalg.eigvalsh(cov)
        eigenvalues = np.sort(eigenvalues)[::-1]  # Sort descending

        # Linearity measure: how much variance is in first component
        total = eigenvalues.sum()
        if total > 1e-10:
            linearity = eigenvalues[0] / total
            if linearity > linearity_threshold:
                mask[i] = False

    return points[mask]


def filter_by_range(points, min_range=0.3, max_range=12.0):
    """
    Remove points too close or too far from origin.
    """
    distances = np.linalg.norm(points, axis=1)
    mask = (distances > min_range) & (distances < max_range)
    return points[mask]


def run_simple_scan(duration=30, name="simple_scan", stationary=True):
    """
    Run a simple scan using IMU quaternion for orientation.

    Args:
        duration: Scan duration in seconds
        name: Name for output file
        stationary: If True, use single reference frame (no per-frame transforms)
                   If False, apply IMU transforms for handheld scanning
    """
    print("=" * 60)
    print("  SIMPLE LIDAR SCAN")
    print("=" * 60)
    print(f"\nDuration: {duration} seconds")
    print(f"Mode: {'STATIONARY' if stationary else 'HANDHELD'}")
    print("Keep sensor STATIONARY for first 2 seconds to calibrate.\n")

    # Connect to LiDAR
    config = UnitreeL2Config()
    lidar = UnitreeL2UDP(config)

    if not lidar.connect():
        print("ERROR: Failed to connect to LiDAR")
        return

    print("[OK] Connected to LiDAR\n")

    # Storage
    all_points = []           # World-frame points
    current_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]

    # Calibration phase
    print("Calibrating IMU (keep still)...")
    cal_start = time.time()
    imu_samples = []

    while time.time() - cal_start < 2.0:
        packet = lidar.receive_packet()
        if packet is None:
            continue

        result = lidar.parse_packet(packet)

        if isinstance(result, dict) and 'quaternion' in result:
            imu_samples.append(result['quaternion'].copy())

    if len(imu_samples) > 0:
        # Use the most recent quaternion as reference
        reference_quaternion = imu_samples[-1]
        current_quaternion = reference_quaternion.copy()
        print(f"[OK] IMU calibrated with {len(imu_samples)} samples")
        print(f"     Reference orientation: [{reference_quaternion[0]:.3f}, "
              f"{reference_quaternion[1]:.3f}, {reference_quaternion[2]:.3f}, "
              f"{reference_quaternion[3]:.3f}]")
    else:
        reference_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        current_quaternion = reference_quaternion.copy()
        print("[WARN] No IMU data received, using identity orientation")

    # Compute reference rotation matrix once for stationary mode
    R_reference = quaternion_to_rotation_matrix(reference_quaternion)

    print()

    # Scanning phase
    if stationary:
        print("Scanning... Keep sensor STATIONARY.")
    else:
        print("Scanning... Move the sensor SLOWLY.")
    print("-" * 60)

    start_time = time.time()
    point_count = 0
    scan_count = 0
    last_print = start_time

    try:
        while time.time() - start_time < duration:
            packet = lidar.receive_packet()
            if packet is None:
                continue

            result = lidar.parse_packet(packet)

            # IMU data - update orientation (only used in handheld mode)
            if isinstance(result, dict) and 'quaternion' in result:
                if not stationary:
                    current_quaternion = result['quaternion']

            # Point cloud data
            elif isinstance(result, tuple) and len(result) == 2:
                points, intensities = result

                if len(points) < 10:
                    continue

                scan_count += 1

                if stationary:
                    # STATIONARY MODE: Use fixed reference frame
                    # All points go directly into the same coordinate frame
                    # The LiDAR's internal rotation handles the 360° coverage
                    points_world = (R_reference @ points.T).T
                else:
                    # HANDHELD MODE: Transform each frame using current IMU orientation
                    R = quaternion_to_rotation_matrix(current_quaternion)
                    points_world = (R @ points.T).T

                # Add to accumulator
                all_points.append(points_world)
                point_count += len(points)

                # Print progress
                if time.time() - last_print > 1.0:
                    elapsed = int(time.time() - start_time)
                    print(f"[{elapsed:3d}s] Scans: {scan_count:4d} | "
                          f"Points: {point_count:,}")
                    last_print = time.time()

    except KeyboardInterrupt:
        print("\nStopped by user")

    finally:
        lidar.disconnect()

    print("-" * 60)
    print()

    # Process results
    if len(all_points) == 0:
        print("No points captured!")
        return

    # Combine all points
    print("Processing points...")
    combined = np.vstack(all_points)
    print(f"  Raw points: {len(combined):,}")

    # Filter by range (remove points too close or too far)
    filtered = filter_by_range(combined, min_range=0.3, max_range=10.0)
    print(f"  After range filter: {len(filtered):,}")

    # Downsample to remove duplicates
    downsampled = voxel_downsample(filtered, voxel_size=0.02)
    print(f"  After dedup: {len(downsampled):,}")

    # Remove statistical outliers (isolated noisy points)
    print("  Removing statistical outliers...")
    no_outliers = statistical_outlier_removal(downsampled, k=20, std_ratio=2.0)
    print(f"  After outlier removal: {len(no_outliers):,}")

    # Remove sparse points (points without enough neighbors)
    print("  Removing sparse points...")
    dense = remove_sparse_points(no_outliers, radius=0.08, min_neighbors=4)
    print(f"  After density filter: {len(dense):,}")

    # Remove linear structures (streaks/lines)
    print("  Removing linear streaks...")
    cleaned = remove_linear_structures(dense, k=8, linearity_threshold=0.85)
    print(f"  After line removal: {len(cleaned):,}")

    # Center for viewing
    center = cleaned.mean(axis=0)
    centered = cleaned - center

    # Calculate bounds
    mins = centered.min(axis=0)
    maxs = centered.max(axis=0)
    dimensions = maxs - mins

    print(f"  Dimensions: {dimensions[0]:.2f} x {dimensions[1]:.2f} x {dimensions[2]:.2f} m")

    # Save for web viewer
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{name}_{timestamp}.json"

    web_models = Path("web/models")
    web_models.mkdir(parents=True, exist_ok=True)

    model_data = {
        'vertices': centered.flatten().tolist(),
        'faces': []  # Point cloud, no mesh
    }

    output_path = web_models / filename
    with open(output_path, 'w') as f:
        json.dump(model_data, f)

    print(f"\n[OK] Saved: {output_path}")
    print(f"     Points: {len(cleaned):,}")
    print(f"\nView at: http://localhost:8000/viewer.html")

    # Update index
    update_index(filename, name)

    return output_path


def update_index(filename, name):
    """Add scan to web viewer index."""
    index_file = Path("web/models/index.json")

    try:
        if index_file.exists():
            with open(index_file, 'r') as f:
                index = json.load(f)
        else:
            index = {'scans': []}

        # Add at beginning
        new_entry = {'name': Path(filename).stem, 'file': filename}

        # Check if already exists
        existing = [s['file'] for s in index['scans']]
        if filename not in existing:
            index['scans'].insert(0, new_entry)

            with open(index_file, 'w') as f:
                json.dump(index, f, indent=2)

    except Exception as e:
        print(f"Warning: Could not update index: {e}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Simple LiDAR scan")
    parser.add_argument('--duration', type=int, default=30,
                       help='Scan duration in seconds')
    parser.add_argument('--name', type=str, default='simple_scan',
                       help='Name for this scan')
    parser.add_argument('--handheld', action='store_true',
                       help='Enable handheld mode (apply IMU transforms per frame)')

    args = parser.parse_args()

    run_simple_scan(
        duration=args.duration,
        name=args.name,
        stationary=not args.handheld
    )
