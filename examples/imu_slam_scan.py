"""
IMU-based SLAM scanning - uses orientation data to properly align scans
Tracks sensor rotation and orientation during movement
"""
import sys
from pathlib import Path
import numpy as np
import logging
import time
from datetime import datetime
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lidar_interface.unitree_l2_udp import UnitreeL2UDP
from src.data_processing.mesh_generator import MeshGenerator

logging.basicConfig(level=logging.WARNING)  # Reduce noise

print("=" * 70)
print("IMU SLAM SCAN - Unitree L2 LiDAR")
print("=" * 70)
print()

# Scan parameters
scan_duration = int(input("Scan duration in seconds (default: 30): ").strip() or "30")
room_name = input("Enter room name (default: 'imu_room'): ").strip() or "imu_room"

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_name = f"{room_name}_{timestamp}"

print()
print(f"Room: {room_name}")
print(f"Duration: {scan_duration}s")
print(f"Mode: IMU-based orientation tracking")
print()
print("INSTRUCTIONS:")
print("  - Walk slowly and smoothly around the room")
print("  - Keep sensor pointed forward")
print("  - Try to maintain steady movement")
print()

# Connect
print(f"[1/5] Connecting and calibrating...")
receiver = UnitreeL2UDP()

if not receiver.connect():
    print("ERROR: Could not connect to sensor!")
    sys.exit(1)

# Calibrate - get initial IMU orientation
print("      Finding initial orientation...")
initial_imu = None
for _ in range(100):  # Try up to 100 packets
    packet = receiver.receive_packet()
    if packet:
        result = receiver.parse_packet(packet)
        if result and isinstance(result, dict) and 'quaternion' in result:
            initial_imu = result
            break

if initial_imu is None:
    print("      Warning: No IMU data found, proceeding without orientation tracking")
    use_imu = False
else:
    print(f"      Initial orientation: {initial_imu['quaternion']}")
    use_imu = True

print()
print(f"[2/5] Scanning for {scan_duration} seconds...")
print()

all_points = []
all_transforms = []
current_orientation = None
packet_count = 0
imu_count = 0

start_time = time.time()
last_print = start_time

try:
    while (time.time() - start_time) < scan_duration:
        packet = receiver.receive_packet()

        if packet is None:
            continue

        result = receiver.parse_packet(packet)

        if result is None:
            continue

        # Check if it's IMU data
        if isinstance(result, dict) and 'quaternion' in result:
            # Update current orientation from IMU
            current_orientation = result
            imu_count += 1
            continue

        # It's point cloud data
        points, intensities = result

        if len(points) < 10:
            continue

        # Transform points based on current orientation
        if use_imu and current_orientation is not None:
            # Convert quaternion to rotation matrix
            quat = current_orientation['quaternion']  # [w, x, y, z]
            rot = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])  # scipy uses [x,y,z,w]
            rotation_matrix = rot.as_matrix()

            # Apply rotation to points
            points_transformed = points @ rotation_matrix.T

            all_points.append(points_transformed)
        else:
            # No IMU, just use raw points
            all_points.append(points)

        packet_count += 1

        # Progress
        current_time = time.time()
        if current_time - last_print >= 3.0:
            elapsed = current_time - start_time
            total_points = sum(len(p) for p in all_points)
            progress = (elapsed / scan_duration) * 100

            status = f"  {elapsed:.0f}s / {scan_duration}s ({progress:.1f}%)"
            status += f" - Points: {packet_count} clouds, {total_points:,} pts"
            if use_imu:
                status += f", IMU: {imu_count} updates"
            print(status)
            last_print = current_time

except KeyboardInterrupt:
    print("\nScan interrupted")

receiver.disconnect()

if len(all_points) == 0:
    print("ERROR: No points captured!")
    sys.exit(1)

# Merge
merged_points = np.vstack(all_points)
total_points = len(merged_points)

print()
print(f"[3/5] Scan complete!")
print(f"      Point clouds: {packet_count}")
print(f"      Total points: {total_points:,}")
if use_imu:
    print(f"      IMU updates: {imu_count}")
    print(f"      Orientation tracking: ENABLED")
else:
    print(f"      Orientation tracking: DISABLED (no IMU data)")
print()

# Process
print(f"[4/5] Processing...")

# Voxel downsample to remove duplicates
if total_points > 100000:
    voxel_size = 0.02
    voxel_indices = np.floor(merged_points / voxel_size).astype(int)
    _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
    merged_points = merged_points[unique_indices]
    print(f"      Voxel filtered: {len(merged_points):,} points (2cm grid)")

# Further downsample if needed
target = 100000
if len(merged_points) > target:
    factor = len(merged_points) // target
    merged_points = merged_points[::factor]
    print(f"      Downsampled: {len(merged_points):,} points")

print()

# Analyze
coords_min = merged_points.min(axis=0)
coords_max = merged_points.max(axis=0)
coords_range = coords_max - coords_min

print(f"      X: {coords_min[0]:.2f}m to {coords_max[0]:.2f}m ({coords_range[0]:.2f}m)")
print(f"      Y: {coords_min[1]:.2f}m to {coords_max[1]:.2f}m ({coords_range[1]:.2f}m)")
print(f"      Z: {coords_min[2]:.2f}m to {coords_max[2]:.2f}m ({coords_range[2]:.2f}m)")
print()

# Generate mesh
print(f"[5/5] Generating mesh...")
mesh_gen = MeshGenerator()
mesh_data = mesh_gen.create_trail_mesh(merged_points, method='delaunay')
print(f"      Mesh: {mesh_data['num_vertices']:,} vertices, {mesh_data['num_faces']:,} faces")
print()

# Save
save_dir = Path(__file__).parent.parent / 'data' / 'imu_slam' / output_name
save_dir.mkdir(parents=True, exist_ok=True)

np.save(save_dir / 'points.npy', merged_points)

# Export
print("      Exporting...")
web_dir = Path(__file__).parent.parent / 'web' / 'models'
web_dir.mkdir(parents=True, exist_ok=True)

json_path = web_dir / f'{output_name}.json'
mesh_gen.export_to_json(mesh_data, str(json_path))
print(f"        Web: {json_path.name}")

mesh_gen.export_to_obj(mesh_data, str(save_dir / f'{output_name}.obj'))
mesh_gen.export_to_ply(mesh_data, str(save_dir / f'{output_name}.ply'))

# Metadata
import json
metadata = {
    "room_name": room_name,
    "scan_date": datetime.now().isoformat(),
    "scan_duration_seconds": scan_duration,
    "total_points": int(total_points),
    "processed_points": int(len(merged_points)),
    "mesh_vertices": int(mesh_data['num_vertices']),
    "mesh_faces": int(mesh_data['num_faces']),
    "imu_enabled": use_imu,
    "imu_updates": imu_count if use_imu else 0,
    "coordinate_range": {
        "x_min": float(coords_min[0]),
        "x_max": float(coords_max[0]),
        "y_min": float(coords_min[1]),
        "y_max": float(coords_max[1]),
        "z_min": float(coords_min[2]),
        "z_max": float(coords_max[2])
    }
}

with open(save_dir / 'metadata.json', 'w') as f:
    json.dump(metadata, f, indent=2)

print()
print("=" * 70)
print("SUCCESS!")
print("=" * 70)
print(f"IMU SLAM scan complete: {output_name}")
print()
if use_imu:
    print("✓ Orientation tracking was ACTIVE")
    print("  Points were rotated based on sensor orientation")
else:
    print("⚠ Orientation tracking was INACTIVE")
    print("  Consider checking IMU packet reception")
print()
print("View: http://localhost:8000/viewer.html")
print(f"Select: {output_name}")
print("=" * 70)