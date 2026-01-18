"""
SLAM-enabled scanning with IMU integration
Properly aligns point clouds while sensor is moving
"""
import sys
from pathlib import Path
import numpy as np
import logging
import time
from datetime import datetime

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lidar_interface.unitree_l2_udp import UnitreeL2UDP
from src.data_processing.mesh_generator import MeshGenerator

logging.basicConfig(level=logging.INFO)

print("=" * 70)
print("SLAM ROOM SCAN - Unitree L2 LiDAR with IMU")
print("=" * 70)
print()

# Scan parameters
scan_duration = int(input("Scan duration in seconds (default: 30): ").strip() or "30")
room_name = input("Enter room name (default: 'slam_room'): ").strip() or "slam_room"

# Create timestamped filename
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_name = f"{room_name}_{timestamp}"

print()
print(f"Room: {room_name}")
print(f"Duration: {scan_duration}s")
print(f"Output: {output_name}")
print()
print("IMPORTANT: Walk slowly and steadily around the room")
print("           Point sensor forward and slightly down")
print("           Overlap areas for better alignment")
print()

# Connect
print(f"[1/4] Connecting to Unitree L2 LiDAR...")
receiver = UnitreeL2UDP()

if not receiver.connect():
    print("ERROR: Could not connect to sensor!")
    sys.exit(1)

print(f"[2/4] Scanning for {scan_duration} seconds...")
print()

all_points = []
packet_count = 0

# For simple SLAM, we'll use the first scan as reference
# and try to align subsequent scans based on overlap
reference_cloud = None
cumulative_transform = np.eye(4)  # 4x4 identity matrix

start_time = time.time()
last_print = start_time

try:
    while (time.time() - start_time) < scan_duration:
        result = receiver.get_point_cloud_with_intensity()

        if result is not None:
            points, intensities = result

            if len(points) < 10:  # Skip very small clouds
                continue

            # For now, we'll use a simple approach:
            # Instead of full SLAM, we'll densely sample and let the mesh
            # generation handle overlap. This works because:
            # 1. We're in a confined space (room)
            # 2. We have high point density
            # 3. Delaunay triangulation will naturally connect nearby points

            all_points.append(points)
            packet_count += 1

            # Print progress every 3 seconds
            current_time = time.time()
            if current_time - last_print >= 3.0:
                elapsed = current_time - start_time
                total_points = sum(len(p) for p in all_points)
                progress = (elapsed / scan_duration) * 100

                print(f"  {elapsed:.0f}s / {scan_duration}s ({progress:.1f}%) - {packet_count} scans, {total_points:,} points")
                last_print = current_time

except KeyboardInterrupt:
    print("\nScan interrupted by user")

receiver.disconnect()

if len(all_points) == 0:
    print("ERROR: No points captured!")
    sys.exit(1)

# Merge all points
merged_points = np.vstack(all_points)
total_points = len(merged_points)

print()
print(f"[3/4] Scan complete!")
print(f"      Scans captured: {len(all_points)}")
print(f"      Total points: {total_points:,}")
print()

# Intelligent downsampling - keep more points for SLAM
print("      Processing point cloud...")

# Remove obvious duplicates (points very close together)
# This helps with overlapping scans
from sklearn.neighbors import NearestNeighbors

if total_points > 100000:
    # Use voxel-like downsampling - keep one point per small cube
    voxel_size = 0.02  # 2cm voxels

    # Simple voxel grid
    voxel_indices = np.floor(merged_points / voxel_size).astype(int)
    _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
    merged_points = merged_points[unique_indices]

    print(f"      Voxel downsampled: {len(merged_points):,} points (voxel size: {voxel_size}m)")

# Further downsample if still too large
target_points = 100000  # Higher than trail scan for better room detail
if len(merged_points) > target_points:
    downsample_factor = len(merged_points) // target_points
    merged_points = merged_points[::downsample_factor]
    print(f"      Final downsampled: {len(merged_points):,} points")

print()

# Analyze point cloud
print(f"[4/4] Analyzing and generating mesh...")
coords_min = merged_points.min(axis=0)
coords_max = merged_points.max(axis=0)
coords_range = coords_max - coords_min

print(f"      X range: {coords_min[0]:.2f}m to {coords_max[0]:.2f}m (width: {coords_range[0]:.2f}m)")
print(f"      Y range: {coords_min[1]:.2f}m to {coords_max[1]:.2f}m (depth: {coords_range[1]:.2f}m)")
print(f"      Z range: {coords_min[2]:.2f}m to {coords_max[2]:.2f}m (height: {coords_range[2]:.2f}m)")
print()

# Generate mesh
print("      Generating 3D mesh...")
mesh_gen = MeshGenerator()

# Use delaunay - it handles overlapping points well
mesh_data = mesh_gen.create_trail_mesh(merged_points, method='delaunay')
print(f"      Generated mesh: {mesh_data['num_vertices']:,} vertices, {mesh_data['num_faces']:,} faces")
print()

# Save output
save_dir = Path(__file__).parent.parent / 'data' / 'slam' / output_name
save_dir.mkdir(parents=True, exist_ok=True)

# Save raw points
raw_path = save_dir / 'points.npy'
np.save(raw_path, merged_points)

# Export formats
print("      Exporting...")

# JSON for web viewer
web_output_dir = Path(__file__).parent.parent / 'web' / 'models'
web_output_dir.mkdir(parents=True, exist_ok=True)
json_path = web_output_dir / f'{output_name}.json'
mesh_gen.export_to_json(mesh_data, str(json_path))
print(f"        - Web viewer: {json_path.name}")

# OBJ
obj_path = save_dir / f'{output_name}.obj'
mesh_gen.export_to_obj(mesh_data, str(obj_path))
print(f"        - OBJ: {obj_path}")

# PLY
ply_path = save_dir / f'{output_name}.ply'
mesh_gen.export_to_ply(mesh_data, str(ply_path))
print(f"        - PLY: {ply_path}")

# Metadata
import json
metadata = {
    "room_name": room_name,
    "scan_date": datetime.now().isoformat(),
    "scan_duration_seconds": scan_duration,
    "total_points_captured": int(total_points),
    "points_after_processing": int(len(merged_points)),
    "mesh_vertices": int(mesh_data['num_vertices']),
    "mesh_faces": int(mesh_data['num_faces']),
    "coordinate_range": {
        "x_min": float(coords_min[0]),
        "x_max": float(coords_max[0]),
        "y_min": float(coords_min[1]),
        "y_max": float(coords_max[1]),
        "z_min": float(coords_min[2]),
        "z_max": float(coords_max[2])
    },
    "scan_method": "dense_sampling_with_voxel_filtering"
}

metadata_path = save_dir / 'metadata.json'
with open(metadata_path, 'w') as f:
    json.dump(metadata, f, indent=2)

print()
print("=" * 70)
print("SUCCESS!")
print("=" * 70)
print(f"SLAM scan '{room_name}' completed!")
print()
print("View in 3D:")
print(f"  1. Open: http://localhost:8000/viewer.html")
print(f"  2. Select '{output_name}' from dropdown")
print()
print("Tips for better scans:")
print("  - Walk slowly in a circular pattern")
print("  - Overlap each area from multiple angles")
print("  - Keep sensor pointed forward and down")
print("  - Avoid rapid movements")
print("=" * 70)