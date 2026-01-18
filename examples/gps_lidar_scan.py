"""
GPS + LiDAR Outdoor Scanning
Combines RTK-GPS positioning with KISS-ICP odometry for accurate outdoor mapping
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
from src.positioning.gps_rtk import GPSRTK

print("="*70)
print("GPS + LiDAR OUTDOOR SCANNING")
print("="*70)
print()
print("Combines RTK-GPS with LiDAR for accurate outdoor 3D mapping")
print()
print("Requirements:")
print("  - SparkFun GPS-RTK2 (ZED-F9P) connected to Raspberry Pi")
print("  - NTRIP RTK correction service (optional, for cm accuracy)")
print("  - Outdoor location with clear sky view")
print()

# Configuration
gps_port = input("GPS serial port (default: /dev/serial0): ").strip() or "/dev/serial0"
scan_duration = int(input("Scan duration in seconds (default: 60): ").strip() or "60")
scan_name = input("Enter scan name (default: 'outdoor_scan'): ").strip() or "outdoor_scan"
require_rtk = input("Require RTK fix? (y/n, default: n): ").strip().lower() == 'y'

print()
print(f"Scan: {scan_name}")
print(f"Duration: {scan_duration}s")
print(f"GPS Port: {gps_port}")
print(f"RTK Required: {require_rtk}")
print()

# Create output directory
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
scan_id = f"{scan_name}_{timestamp}"
output_dir = Path(__file__).parent.parent / "data" / "gps_lidar" / scan_id
output_dir.mkdir(parents=True, exist_ok=True)

print(f"[1/5] Connecting to GPS...")

try:
    gps = GPSRTK(port=gps_port)
    print(f"  GPS connected on {gps_port}")
except Exception as e:
    print(f"ERROR: Could not connect to GPS: {e}")
    print("Check:")
    print("  - GPS module is connected")
    print("  - Correct serial port")
    print("  - UART enabled on Raspberry Pi")
    sys.exit(1)

print()
print(f"[2/5] Waiting for GPS fix...")

if not gps.wait_for_fix(timeout=120, rtk_required=require_rtk):
    print("ERROR: Could not acquire GPS fix!")
    print("Make sure you have:")
    print("  - Clear view of sky")
    print("  - GPS antenna properly connected")
    print("  - NTRIP correction service (for RTK)")
    gps.close()
    sys.exit(1)

# Get origin position
origin_position = gps.get_position()
if not origin_position:
    print("ERROR: No GPS position available!")
    gps.close()
    sys.exit(1)

origin_lat = origin_position['latitude']
origin_lon = origin_position['longitude']
origin_alt = origin_position['altitude']

print()
print(f"Origin Position:")
print(f"  Lat: {origin_lat:.8f}°")
print(f"  Lon: {origin_lon:.8f}°")
print(f"  Alt: {origin_alt:.2f}m")
print(f"  Quality: {gps.get_fix_quality_string()}")
print(f"  Satellites: {origin_position['num_satellites']}")

print()
print(f"[3/5] Connecting to LiDAR...")

receiver = UnitreeL2UDP()
if not receiver.connect():
    print("ERROR: Could not connect to LiDAR!")
    gps.close()
    sys.exit(1)

print(f"[4/5] Initializing KISS-ICP...")

odometry = KISSICPOdometry(voxel_size=0.02, max_range=50.0)  # Larger range for outdoor

print(f"[5/5] Scanning with GPS + LiDAR fusion...")
print()
print("SCANNING TIPS:")
print("  - Move slowly and steadily")
print("  - GPS updates position every 1-2 seconds")
print("  - KISS-ICP fills in between GPS updates")
print("  - Best results: walk in systematic pattern")
print()

all_transformed_points = []
poses = []
gps_positions = []
fusion_positions = []

start_time = time.time()
last_print = start_time
last_process = start_time
last_gps = start_time

scan_count = 0
process_interval = 0.2  # Process LiDAR every 0.2s
gps_interval = 1.0      # Read GPS every 1s

# Current position estimate
current_gps_pos = np.array([0.0, 0.0, 0.0])
last_icp_pos = np.array([0.0, 0.0, 0.0])

try:
    while (time.time() - start_time) < scan_duration:
        current_time = time.time()

        # Update GPS position
        if current_time - last_gps >= gps_interval:
            gps_data = gps.get_position()
            if gps_data:
                # Convert to local coordinates
                local_coords = gps.get_local_coordinates(origin_lat, origin_lon)
                if local_coords:
                    current_gps_pos = np.array(local_coords)
                    gps_positions.append({
                        'timestamp': current_time,
                        'position': current_gps_pos.copy(),
                        'quality': gps.get_fix_quality_string(),
                        'satellites': gps_data['num_satellites']
                    })
            last_gps = current_time

        # Process LiDAR
        result = receiver.get_point_cloud_with_intensity()

        if result is None:
            continue

        if isinstance(result, dict):  # Skip IMU packets
            continue

        points, intensities = result
        scan_count += 1

        # Process with KISS-ICP
        if current_time - last_process >= process_interval:
            transformed_points, pose, success = odometry.process_scan(points)

            # Get ICP position estimate
            icp_pos = pose[:3, 3]

            # Fuse GPS and ICP
            # GPS provides absolute position, ICP provides relative movement
            # Simple fusion: GPS position + ICP delta from last GPS reading
            icp_delta = icp_pos - last_icp_pos
            fused_pos = current_gps_pos + icp_delta

            # Create fused transformation matrix
            fused_pose = pose.copy()
            fused_pose[:3, 3] = fused_pos

            # Transform points with fused pose
            points_homogeneous = np.hstack([points, np.ones((len(points), 1))])
            transformed = (fused_pose @ points_homogeneous.T).T
            final_points = transformed[:, :3]

            all_transformed_points.append(final_points)
            poses.append(fused_pose)
            fusion_positions.append(fused_pos.copy())

            last_icp_pos = icp_pos.copy()
            last_process = current_time

        # Progress update
        if current_time - last_print >= 2.0:
            elapsed = current_time - start_time
            progress = (elapsed / scan_duration) * 100

            print(f"  {int(elapsed)}s / {scan_duration}s ({int(progress)}%)")
            print(f"    LiDAR scans: {scan_count}")
            print(f"    GPS updates: {len(gps_positions)}")
            print(f"    Current pos: ({fused_pos[0]:.1f}, {fused_pos[1]:.1f}, {fused_pos[2]:.1f})m")
            print(f"    GPS quality: {gps.get_fix_quality_string()}")
            print()

            last_print = current_time

except KeyboardInterrupt:
    print("\nScan interrupted by user")

finally:
    receiver.disconnect()
    gps.close()

print()
print(f"Processing results...")

if len(all_transformed_points) == 0:
    print("ERROR: No point clouds captured!")
    sys.exit(1)

# Merge and process
merged_points = np.vstack(all_transformed_points)
total_points = len(merged_points)

print(f"  Total points: {total_points:,}")

# Voxel downsampling
voxel_size = 0.05  # 5cm for outdoor
voxel_dict = {}

for point in merged_points:
    voxel_key = tuple((point / voxel_size).astype(int))
    if voxel_key not in voxel_dict:
        voxel_dict[voxel_key] = point

merged_points = np.array(list(voxel_dict.values()))
processed_points = len(merged_points)

print(f"  After voxel filter: {processed_points:,}")

# Save data
np.save(output_dir / "points.npy", merged_points)
np.save(output_dir / "poses.npy", np.array(poses))

# Save GPS track
with open(output_dir / "gps_track.json", 'w') as f:
    json.dump({
        'origin': {'lat': origin_lat, 'lon': origin_lon, 'alt': origin_alt},
        'positions': [{
            'timestamp': p['timestamp'],
            'x': float(p['position'][0]),
            'y': float(p['position'][1]),
            'z': float(p['position'][2]),
            'quality': p['quality'],
            'satellites': p['satellites']
        } for p in gps_positions]
    }, f, indent=2)

# Generate mesh
print("  Generating mesh...")
mesh_gen = MeshGenerator()
mesh_data = mesh_gen.create_trail_mesh(merged_points, method='delaunay')

if mesh_data:
    print(f"  Mesh: {mesh_data['num_vertices']:,} vertices")

    # Export
    web_output_dir = Path(__file__).parent.parent / "web" / "models"
    web_output_dir.mkdir(parents=True, exist_ok=True)

    mesh_gen.export_to_json(mesh_data, str(web_output_dir / f"{scan_id}.json"))
    mesh_gen.export_to_obj(mesh_data, str(output_dir / f"{scan_id}.obj"))
    mesh_gen.export_to_ply(mesh_data, str(output_dir / f"{scan_id}.ply"))

    # Metadata
    metadata = {
        "scan_name": scan_name,
        "scan_date": datetime.now().isoformat(),
        "scan_duration_seconds": scan_duration,
        "method": "gps_lidar_fusion",
        "origin_latitude": origin_lat,
        "origin_longitude": origin_lon,
        "origin_altitude": origin_alt,
        "gps_updates": len(gps_positions),
        "lidar_scans": scan_count,
        "total_points": total_points,
        "processed_points": processed_points,
        "mesh_vertices": int(mesh_data['num_vertices']),
        "mesh_faces": int(mesh_data['num_faces'])
    }

    with open(output_dir / "metadata.json", 'w') as f:
        json.dump(metadata, f, indent=2)

print()
print("="*70)
print("GPS + LiDAR SCAN COMPLETE!")
print("="*70)
print()
print(f"Output: {output_dir}")
print(f"GPS updates: {len(gps_positions)}")
print(f"Web viewer: http://localhost:8000/viewer.html")
print("="*70)