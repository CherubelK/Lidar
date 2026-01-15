"""
Quick room scan with optimized processing for dense point clouds
"""
import sys
from pathlib import Path
import numpy as np
import logging

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lidar_interface.unitree_l2_udp import UnitreeL2UDP
from src.data_processing.mesh_generator import MeshGenerator

logging.basicConfig(level=logging.INFO)

print("=" * 70)
print("QUICK ROOM SCAN - Unitree L2 LiDAR")
print("=" * 70)
print()

# Scan parameters
scan_duration = 10  # seconds
room_name = "quick_scan"

# Connect and scan
print(f"[1/4] Connecting to Unitree L2 LiDAR...")
receiver = UnitreeL2UDP()

if not receiver.connect():
    print("ERROR: Could not connect to sensor!")
    sys.exit(1)

print(f"[2/4] Scanning for {scan_duration} seconds...")
print()

all_points = []
packet_count = 0

import time
start_time = time.time()
last_print = start_time

try:
    while (time.time() - start_time) < scan_duration:
        result = receiver.get_point_cloud_with_intensity()

        if result is not None:
            points, intensities = result
            all_points.append(points)
            packet_count += 1

            # Print progress every second
            if time.time() - last_print >= 1.0:
                elapsed = time.time() - start_time
                total_points = sum(len(p) for p in all_points)
                print(f"  {elapsed:.1f}s - {packet_count} point clouds, {total_points:,} points")
                last_print = time.time()

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
print(f"      Point clouds: {len(all_points)}")
print(f"      Total points: {total_points:,}")
print()

# Simple downsampling - keep every Nth point
downsample_factor = max(1, total_points // 50000)  # Target ~50k points
if downsample_factor > 1:
    print(f"      Downsampling by factor of {downsample_factor}...")
    merged_points = merged_points[::downsample_factor]
    print(f"      Points after downsampling: {len(merged_points):,}")
    print()

# Generate mesh
print(f"[4/4] Generating 3D mesh...")
mesh_gen = MeshGenerator()

# Create mesh using delaunay triangulation
mesh_data = mesh_gen.create_trail_mesh(merged_points, method='delaunay')
print(f"      Generated mesh: {mesh_data['num_vertices']} vertices, {mesh_data['num_faces']} faces")
print()

# Export to JSON for web viewer
output_dir = Path(__file__).parent.parent / 'web' / 'models'
output_dir.mkdir(parents=True, exist_ok=True)
output_path = output_dir / f'{room_name}.json'

mesh_gen.export_to_json(mesh_data, str(output_path))

print("=" * 70)
print("SUCCESS!")
print("=" * 70)
print(f"Mesh saved to: {output_path}")
print()
print("To view:")
print(f"  1. Open: http://localhost:8000/viewer.html")
print(f"  2. Select '{room_name}' from the dropdown")
print("=" * 70)