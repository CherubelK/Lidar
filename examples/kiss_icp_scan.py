"""
Room scanning using KISS-ICP professional SLAM
Better drift correction and loop closure compared to basic ICP
"""
import sys
from pathlib import Path
import time
import numpy as np
from datetime import datetime
import json

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lidar_interface.unitree_l2_udp import UnitreeL2UDP
from src.data_processing.kiss_icp_odometry import KISSICPOdometry
from src.data_processing.mesh_generator import MeshGenerator

print("="*70)
print("KISS-ICP SLAM SCAN - Professional LiDAR Odometry")
print("="*70)
print()
print("KISS-ICP is a modern SLAM framework used in robotics.")
print("Advantages over basic ICP:")
print("  - Adaptive thresholds for better matching")
print("  - Optimized point-to-point registration")
print("  - Better handling of dynamic environments")
print("  - Proven in autonomous vehicles and robots")
print()
print("SCANNING INSTRUCTIONS:")
print("  1. Move SLOWLY and SMOOTHLY (10cm every 2-3 seconds)")
print("  2. Walk systematic pattern: perimeter + internal grid")
print("  3. Cover entire room area, not just linear paths")
print("  4. Return to starting point for loop closure")
print("  5. Keep sensor upright, allow gentle rotation")
print()
print("Best results: Systematic area coverage with return to start")
print()

# Get scan parameters
scan_duration = int(input("Scan duration in seconds (default: 60): ").strip() or "60")
room_name = input("Enter room name (default: 'kiss_room'): ").strip() or "kiss_room"

print()
print(f"Room: {room_name}")
print(f"Duration: {scan_duration}s")
print(f"Method: KISS-ICP Professional SLAM")
print()

# Create output directory
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
scan_name = f"{room_name}_{timestamp}"
output_dir = Path(__file__).parent.parent / "data" / "kiss_icp" / scan_name
output_dir.mkdir(parents=True, exist_ok=True)

print(f"[1/4] Connecting to Unitree L2...")

# Connect to LiDAR
receiver = UnitreeL2UDP()
if not receiver.connect():
    print("ERROR: Could not connect to LiDAR!")
    print("Make sure the sensor is powered and connected to 192.168.1.62")
    sys.exit(1)

print(f"[2/4] Initializing KISS-ICP SLAM...")

# Initialize KISS-ICP odometry
odometry = KISSICPOdometry(
    voxel_size=0.02,  # 2cm voxels
    max_range=10.0    # 10m max range for indoor
)

print(f"[3/4] Scanning with KISS-ICP...")
print()
print("TIP: For best results, return to your starting position at the end!")
print("This enables 'loop closure' - matching start and end to correct drift.")
print()

all_transformed_points = []
all_intensities = []
poses = []

start_time = time.time()
last_print = start_time
last_process = start_time

scan_count = 0
process_interval = 0.2  # Process every 0.2 seconds for dense coverage

try:
    while (time.time() - start_time) < scan_duration:
        result = receiver.get_point_cloud_with_intensity()

        if result is None:
            continue

        # Skip IMU packets
        if isinstance(result, dict):
            continue

        points, intensities = result
        scan_count += 1

        current_time = time.time()

        # Process at regular intervals
        if current_time - last_process < process_interval:
            continue

        last_process = current_time

        # Process with KISS-ICP
        transformed_points, pose, success = odometry.process_scan(points, timestamp=current_time)

        # Always accumulate points (KISS-ICP is more robust)
        all_transformed_points.append(transformed_points)
        all_intensities.append(intensities)
        poses.append(pose.copy())

        # Progress update every 2 seconds
        if current_time - last_print >= 2.0:
            elapsed = current_time - start_time
            progress = (elapsed / scan_duration) * 100

            stats = odometry.get_statistics()
            position = pose[:3, 3]

            print(f"  {int(elapsed)}s / {scan_duration}s ({int(progress)}%)")
            print(f"    Processed: {stats['total_poses']} scans, {sum(len(p) for p in all_transformed_points):,} points")
            print(f"    Position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})m")
            print(f"    Distance traveled: {stats['path_length']:.2f}m")
            print()

            last_print = current_time

except KeyboardInterrupt:
    print("\nScan interrupted by user")

finally:
    receiver.disconnect()

print()
print(f"[4/4] Processing results...")

# Get final statistics
stats = odometry.get_statistics()

print(f"  Total scans captured: {scan_count}")
print(f"  Scans processed: {stats['total_poses']}")
print(f"  Path traveled: {stats['path_length']:.2f}m")
print()

if len(all_transformed_points) == 0:
    print("ERROR: No point clouds captured!")
    sys.exit(1)

# Merge all point clouds
merged_points = np.vstack(all_transformed_points)
total_points = len(merged_points)

print(f"  Total points before filtering: {total_points:,}")

# Voxel downsampling for final mesh
print("  Applying voxel downsampling...")

voxel_size = 0.03  # 3cm voxels for final mesh
voxel_dict = {}

for point in merged_points:
    voxel_key = tuple((point / voxel_size).astype(int))
    if voxel_key not in voxel_dict:
        voxel_dict[voxel_key] = point

merged_points = np.array(list(voxel_dict.values()))
processed_points = len(merged_points)

print(f"  Points after voxel filtering: {processed_points:,}")
print()

# Save point cloud
points_file = output_dir / "points.npy"
np.save(points_file, merged_points)
print(f"  Saved point cloud: {points_file}")

# Save poses
poses_file = output_dir / "poses.npy"
np.save(poses_file, np.array(poses))
print(f"  Saved poses: {poses_file}")

# Generate mesh
print("  Generating mesh...")
mesh_gen = MeshGenerator()
mesh_data = mesh_gen.create_trail_mesh(merged_points, method='delaunay')

if mesh_data is None:
    print("ERROR: Failed to generate mesh!")
    sys.exit(1)

print(f"      Mesh: {mesh_data['num_vertices']:,} vertices, {mesh_data['num_faces']:,} faces")

# Save mesh in web viewer format
web_output_dir = Path(__file__).parent.parent / "web" / "models"
web_output_dir.mkdir(parents=True, exist_ok=True)

web_output_file = web_output_dir / f"{scan_name}.json"
mesh_gen.export_to_json(mesh_data, str(web_output_file))
print(f"  Saved web model: {web_output_file}")

# Export OBJ and PLY
mesh_gen.export_to_obj(mesh_data, str(output_dir / f'{scan_name}.obj'))
mesh_gen.export_to_ply(mesh_data, str(output_dir / f'{scan_name}.ply'))

print(f"  Saved OBJ and PLY formats")
print()

# Save metadata
metadata = {
    "room_name": room_name,
    "scan_date": datetime.now().isoformat(),
    "scan_duration_seconds": scan_duration,
    "method": "kiss_icp_slam",
    "scans_captured": scan_count,
    "scans_processed": stats['total_poses'],
    "total_points": total_points,
    "processed_points": processed_points,
    "mesh_vertices": int(mesh_data['num_vertices']),
    "mesh_faces": int(mesh_data['num_faces']),
    "estimated_path_length_m": stats['path_length'],
    "coordinate_range": {
        "x_min": float(merged_points[:, 0].min()),
        "x_max": float(merged_points[:, 0].max()),
        "y_min": float(merged_points[:, 1].min()),
        "y_max": float(merged_points[:, 1].max()),
        "z_min": float(merged_points[:, 2].min()),
        "z_max": float(merged_points[:, 2].max())
    }
}

metadata_file = output_dir / "metadata.json"
with open(metadata_file, 'w') as f:
    json.dump(metadata, f, indent=2)

print(f"  Saved metadata: {metadata_file}")
print()

print("="*70)
print("KISS-ICP SCAN COMPLETE!")
print("="*70)
print()
print(f"Output directory: {output_dir}")
print(f"Web viewer: http://localhost:8000/viewer.html")
print(f"Select: {scan_name}")
print()
print("KISS-ICP Performance:")
print(f"  Scans processed: {stats['total_poses']}")
print(f"  Path length: {stats['path_length']:.2f}m")
print(f"  Avg movement: {stats['avg_translation']:.3f}m per scan")
print()
print("Compare with basic ICP to see improvement!")
print("="*70)