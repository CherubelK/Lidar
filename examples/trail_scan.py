"""
Trail scanning script optimized for outdoor hiking trail mapping
Includes GPS coordinate tracking (ready for integration) and extended scanning
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
print("TRAIL SCAN - Unitree L2 LiDAR")
print("=" * 70)
print()

# Scan parameters
scan_duration = 60  # seconds - longer for trail segments
trail_name = input("Enter trail name (default: 'trail_scan'): ").strip() or "trail_scan"
scan_distance = input("Walking distance in meters (default: '50'): ").strip() or "50"

# Create timestamped filename
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_name = f"{trail_name}_{timestamp}"

print()
print(f"Trail: {trail_name}")
print(f"Distance: {scan_distance}m")
print(f"Duration: {scan_duration}s")
print(f"Output: {output_name}")
print()

# GPS coordinates (placeholder for future GPS integration)
# TODO: Integrate with GPS module to capture actual coordinates
gps_start = {
    "lat": None,
    "lon": None,
    "alt": None,
    "note": "GPS integration pending"
}

# Connect and scan
print(f"[1/5] Connecting to Unitree L2 LiDAR...")
receiver = UnitreeL2UDP()

if not receiver.connect():
    print("ERROR: Could not connect to sensor!")
    sys.exit(1)

print(f"[2/5] Scanning for {scan_duration} seconds...")
print("      Start walking along the trail at steady pace")
print("      Hold sensor pointed forward and slightly down")
print()

all_points = []
all_intensities = []
packet_count = 0

start_time = time.time()
last_print = start_time
last_save = start_time

# Create incremental save directory
save_dir = Path(__file__).parent.parent / 'data' / 'trails' / output_name
save_dir.mkdir(parents=True, exist_ok=True)

try:
    while (time.time() - start_time) < scan_duration:
        result = receiver.get_point_cloud_with_intensity()

        if result is not None:
            points, intensities = result
            all_points.append(points)
            all_intensities.append(intensities)
            packet_count += 1

            # Print progress every 5 seconds
            current_time = time.time()
            if current_time - last_print >= 5.0:
                elapsed = current_time - start_time
                total_points = sum(len(p) for p in all_points)
                progress = (elapsed / scan_duration) * 100

                print(f"  {elapsed:.0f}s / {scan_duration}s ({progress:.1f}%) - {packet_count} scans, {total_points:,} points")
                last_print = current_time

            # Incremental save every 15 seconds (in case of crash)
            if current_time - last_save >= 15.0:
                merged_points = np.vstack(all_points)
                checkpoint_path = save_dir / f'checkpoint_{int(elapsed)}s.npy'
                np.save(checkpoint_path, merged_points)
                print(f"      Checkpoint saved: {len(merged_points):,} points")
                last_save = current_time

except KeyboardInterrupt:
    print("\nScan interrupted by user")

receiver.disconnect()

if len(all_points) == 0:
    print("ERROR: No points captured!")
    sys.exit(1)

# Merge all points
merged_points = np.vstack(all_points)
merged_intensities = np.hstack(all_intensities)
total_points = len(merged_points)

print()
print(f"[3/5] Scan complete!")
print(f"      Scans captured: {len(all_points)}")
print(f"      Total points: {total_points:,}")
print()

# Save raw point cloud
raw_path = save_dir / 'raw_points.npy'
intensity_path = save_dir / 'intensities.npy'
np.save(raw_path, merged_points)
np.save(intensity_path, merged_intensities)
print(f"      Raw data saved: {raw_path}")

# Adaptive downsampling based on point count
if total_points > 1000000:
    # Very large scan - aggressive downsampling
    target_points = 75000
    print(f"      Large scan detected - targeting {target_points} points")
elif total_points > 500000:
    # Medium scan
    target_points = 60000
    print(f"      Medium scan - targeting {target_points} points")
else:
    # Small scan - lighter downsampling
    target_points = 50000
    print(f"      Small scan - targeting {target_points} points")

downsample_factor = max(1, total_points // target_points)
if downsample_factor > 1:
    print(f"      Downsampling by factor of {downsample_factor}...")
    merged_points = merged_points[::downsample_factor]
    merged_intensities = merged_intensities[::downsample_factor]
    print(f"      Points after downsampling: {len(merged_points):,}")
    print()

# Analyze point cloud
print(f"[4/5] Analyzing point cloud...")
coords_min = merged_points.min(axis=0)
coords_max = merged_points.max(axis=0)
coords_range = coords_max - coords_min

print(f"      X range: {coords_min[0]:.2f}m to {coords_max[0]:.2f}m (width: {coords_range[0]:.2f}m)")
print(f"      Y range: {coords_min[1]:.2f}m to {coords_max[1]:.2f}m (depth: {coords_range[1]:.2f}m)")
print(f"      Z range: {coords_min[2]:.2f}m to {coords_max[2]:.2f}m (height: {coords_range[2]:.2f}m)")
print()

# Generate mesh
print(f"[5/5] Generating 3D mesh...")
mesh_gen = MeshGenerator()

# Create mesh using delaunay triangulation
mesh_data = mesh_gen.create_trail_mesh(merged_points, method='delaunay')
print(f"      Generated mesh: {mesh_data['num_vertices']:,} vertices, {mesh_data['num_faces']:,} faces")
print()

# Export to multiple formats
print("      Exporting to formats:")

# JSON for web viewer
web_output_dir = Path(__file__).parent.parent / 'web' / 'models'
web_output_dir.mkdir(parents=True, exist_ok=True)
json_path = web_output_dir / f'{output_name}.json'
mesh_gen.export_to_json(mesh_data, str(json_path))
print(f"        - JSON: {json_path}")

# OBJ for 3D software
obj_path = save_dir / f'{output_name}.obj'
mesh_gen.export_to_obj(mesh_data, str(obj_path))
print(f"        - OBJ:  {obj_path}")

# PLY for point cloud software
ply_path = save_dir / f'{output_name}.ply'
mesh_gen.export_to_ply(mesh_data, str(ply_path))
print(f"        - PLY:  {ply_path}")

# Save metadata
metadata = {
    "trail_name": trail_name,
    "scan_date": datetime.now().isoformat(),
    "scan_duration_seconds": scan_duration,
    "planned_distance_meters": scan_distance,
    "total_points_captured": int(total_points),
    "points_after_downsampling": int(len(merged_points)),
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
    "gps_start": gps_start
}

import json
metadata_path = save_dir / 'metadata.json'
with open(metadata_path, 'w') as f:
    json.dump(metadata, f, indent=2)

print(f"        - Metadata: {metadata_path}")
print()

print("=" * 70)
print("SUCCESS!")
print("=" * 70)
print(f"Trail scan '{trail_name}' completed successfully!")
print()
print("Data saved to:")
print(f"  {save_dir}")
print()
print("To view in 3D:")
print(f"  1. Start web server: cd web && python -m http.server 8000")
print(f"  2. Open: http://localhost:8000/viewer.html")
print(f"  3. Select '{output_name}' from the dropdown")
print()
print("Next steps:")
print("  - Review the 3D scan for quality")
print("  - Combine multiple scans for longer trails")
print("  - Add GPS coordinates for georeferencing")
print("=" * 70)