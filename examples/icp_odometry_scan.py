"""
ICP-based LiDAR Odometry Scanning
Estimates sensor movement by matching consecutive point clouds
IMPORTANT: Move VERY SLOWLY for best results
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
from src.data_processing.icp_registration import LiDAROdometry

# Reduce log noise
logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)

print("=" * 70)
print("ICP ODOMETRY SCAN - Unitree L2 LiDAR")
print("=" * 70)
print()
print("This uses point cloud matching to estimate your movement.")
print()
print("CRITICAL INSTRUCTIONS FOR ROOM SCANNING:")
print("  1. Move VERY SLOWLY (like 5-10cm every 2 seconds)")
print("  2. Walk in a systematic pattern (e.g., perimeter, then zigzag)")
print("  3. Cover the room area, not just one path - move in multiple directions")
print("  4. Keep sensor roughly upright and rotating slowly as you walk")
print("  5. Avoid blank walls - ICP needs features to match")
print("  6. For a complete room, walk around ALL walls and through the center")
print()
print("SCANNING STRATEGY:")
print("  - Start in one corner")
print("  - Walk slowly along each wall")
print("  - Then walk across the room in parallel lines")
print("  - This creates overlapping coverage of the entire space")
print()
print("Best results: Rooms with furniture/objects")
print("Worst results: Long empty hallways or single-direction paths")
print()

# Scan parameters
scan_duration = int(input("Scan duration in seconds (default: 20): ").strip() or "20")
room_name = input("Enter room name (default: 'icp_room'): ").strip() or "icp_room"

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_name = f"{room_name}_{timestamp}"

print()
print(f"Room: {room_name}")
print(f"Duration: {scan_duration}s")
print(f"Method: ICP Point Cloud Registration")
print()

# Connect
print(f"[1/4] Connecting...")
receiver = UnitreeL2UDP()

if not receiver.connect():
    print("ERROR: Could not connect to sensor!")
    sys.exit(1)

# Initialize odometry
# Smaller voxel size for more detail, stricter ICP parameters
odometry = LiDAROdometry(voxel_size=0.02)  # 2cm voxels for finer detail
odometry.icp.max_correspondence_distance = 0.25  # Stricter matching (was 0.3)
odometry.icp.max_iterations = 40  # More iterations for better convergence

print(f"[2/4] Scanning with odometry...")
print()
print("REMEMBER: Move VERY SLOWLY!")
print()

all_transformed_points = []
poses = []
successful_registrations = 0
failed_registrations = 0

start_time = time.time()
last_print = start_time
last_process = start_time

scan_count = 0
process_interval = 0.3  # Process every 0.3 seconds for better coverage

try:
    while (time.time() - start_time) < scan_duration:
        result = receiver.get_point_cloud_with_intensity()

        if result is None:
            continue

        # Check if it's IMU data (dict) or point cloud (tuple)
        if isinstance(result, dict):
            continue  # Skip IMU packets

        points, intensities = result
        scan_count += 1

        # Only process every N scans for better overlap
        current_time = time.time()
        if current_time - last_process < process_interval:
            continue

        last_process = current_time

        if len(points) < 50:
            continue

        # Process with ICP odometry
        transformed_points, pose, success = odometry.process_scan(points)

        if success:
            all_transformed_points.append(transformed_points)
            poses.append(pose.copy())
            successful_registrations += 1
        else:
            failed_registrations += 1

        # Progress every 2 seconds
        if current_time - last_print >= 2.0:
            elapsed = current_time - start_time
            total_points = sum(len(p) for p in all_transformed_points)
            progress = (elapsed / scan_duration) * 100

            # Current position from pose
            pos_x, pos_y, pos_z = pose[0, 3], pose[1, 3], pose[2, 3]
            distance_traveled = np.sqrt(pos_x**2 + pos_y**2)

            print(f"  {elapsed:.0f}s / {scan_duration}s ({progress:.0f}%)")
            print(f"    Processed: {len(all_transformed_points)} scans, {total_points:,} points")
            print(f"    Position: ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f})m")
            print(f"    Distance traveled: {distance_traveled:.2f}m")
            print(f"    Success rate: {successful_registrations}/{successful_registrations+failed_registrations}")
            print()

            last_print = current_time

except KeyboardInterrupt:
    print("\nScan interrupted")

receiver.disconnect()

print()
print(f"[3/4] Scan complete!")
print(f"      Total scans captured: {scan_count}")
print(f"      Processed scans: {len(all_transformed_points)}")
print(f"      Successful registrations: {successful_registrations}")
print(f"      Failed registrations: {failed_registrations}")

if successful_registrations == 0:
    print()
    print("ERROR: No successful registrations!")
    print("Tips:")
    print("  - Move slower")
    print("  - Ensure there are objects/features in view")
    print("  - Avoid pointing at blank walls")
    sys.exit(1)

success_rate = successful_registrations / (successful_registrations + failed_registrations)
if success_rate < 0.5:
    print()
    print(f"WARNING: Low success rate ({success_rate:.1%})")
    print("Results may be poor. Consider:")
    print("  - Moving even slower")
    print("  - Ensuring good overlap between scans")
    print("  - Scanning in feature-rich environments")

# Merge
if len(all_transformed_points) == 0:
    print("ERROR: No points to process!")
    sys.exit(1)

merged_points = np.vstack(all_transformed_points)
total_points = len(merged_points)

print(f"      Total points: {total_points:,}")

# Calculate path traveled
if len(poses) > 1:
    path_length = 0
    for i in range(1, len(poses)):
        prev_pos = poses[i-1][:3, 3]
        curr_pos = poses[i][:3, 3]
        path_length += np.linalg.norm(curr_pos - prev_pos)
    print(f"      Estimated path length: {path_length:.2f}m")

print()

# Process
print(f"[4/4] Processing and generating mesh...")

# Voxel downsample to remove duplicates from overlapping scans
if total_points > 100000:
    voxel_size = 0.02
    voxel_indices = np.floor(merged_points / voxel_size).astype(int)
    _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
    merged_points = merged_points[unique_indices]
    print(f"      Voxel filtered: {len(merged_points):,} points")

# Further downsample if needed
target = 100000
if len(merged_points) > target:
    factor = len(merged_points) // target
    merged_points = merged_points[::factor]
    print(f"      Downsampled: {len(merged_points):,} points")

# Analyze
coords_min = merged_points.min(axis=0)
coords_max = merged_points.max(axis=0)
coords_range = coords_max - coords_min

print(f"      X: {coords_min[0]:.2f}m to {coords_max[0]:.2f}m ({coords_range[0]:.2f}m)")
print(f"      Y: {coords_min[1]:.2f}m to {coords_max[1]:.2f}m ({coords_range[1]:.2f}m)")
print(f"      Z: {coords_min[2]:.2f}m to {coords_max[2]:.2f}m ({coords_range[2]:.2f}m)")
print()

# Generate mesh
print("      Generating mesh...")
mesh_gen = MeshGenerator()
mesh_data = mesh_gen.create_trail_mesh(merged_points, method='delaunay')
print(f"      Mesh: {mesh_data['num_vertices']:,} vertices, {mesh_data['num_faces']:,} faces")
print()

# Save
save_dir = Path(__file__).parent.parent / 'data' / 'icp_odometry' / output_name
save_dir.mkdir(parents=True, exist_ok=True)

np.save(save_dir / 'points.npy', merged_points)
np.save(save_dir / 'poses.npy', np.array(poses))

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
    "method": "icp_odometry",
    "scans_captured": scan_count,
    "scans_processed": len(all_transformed_points),
    "successful_registrations": successful_registrations,
    "failed_registrations": failed_registrations,
    "success_rate": success_rate,
    "total_points": int(total_points),
    "processed_points": int(len(merged_points)),
    "mesh_vertices": int(mesh_data['num_vertices']),
    "mesh_faces": int(mesh_data['num_faces']),
    "estimated_path_length_m": float(path_length) if len(poses) > 1 else 0.0,
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
print("ICP ODOMETRY SCAN COMPLETE!")
print("=" * 70)
print(f"Scan: {output_name}")
print(f"Success rate: {success_rate:.1%}")
if success_rate > 0.7:
    print("Status: GOOD - Results should be usable")
elif success_rate > 0.5:
    print("Status: OK - Results may have some drift")
else:
    print("Status: POOR - Consider rescanning slower")
print()
print("View: http://localhost:8000/viewer.html")
print(f"Select: {output_name}")
print()
print("Tips for better results:")
print("  - Move slower (seriously, like glacially slow)")
print("  - Walk in a systematic grid pattern, not just back-and-forth")
print("  - Cover the ENTIRE room area - walk perimeter + internal paths")
print("  - Each LiDAR packet scans 323° around - you need spatial coverage")
print("  - Scan in rooms with furniture/features for better ICP matching")
print("  - Avoid rapid turns or movements")
print()
print("WHY YOUR PREVIOUS SCAN WAS FLAT:")
print("  - Moving in only ONE direction creates a thin ribbon, not a room")
print("  - You need to walk in MULTIPLE directions to capture full 3D space")
print("  - Think: perimeter walk + zigzag through middle = complete coverage")
print("=" * 70)