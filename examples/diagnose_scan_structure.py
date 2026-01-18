"""
Diagnostic script to analyze individual LiDAR scan structure
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lidar_interface.unitree_l2_udp import UnitreeL2UDP

print("="*70)
print("LIDAR SCAN STRUCTURE DIAGNOSTIC")
print("="*70)
print()
print("Capturing 10 individual scans to analyze 3D structure...")
print()

# Connect
receiver = UnitreeL2UDP()
if not receiver.connect():
    print("ERROR: Could not connect!")
    sys.exit(1)

# Capture first 10 scans
scans = []
scan_count = 0

while scan_count < 10:
    result = receiver.get_point_cloud_with_intensity()

    if result is None:
        continue

    # Skip IMU packets
    if isinstance(result, dict):
        continue

    points, intensities = result

    if len(points) < 10:
        continue

    scans.append(points)
    scan_count += 1
    print(f"  Captured scan {scan_count}: {len(points)} points")

receiver.disconnect()

print()
print("="*70)
print("ANALYSIS")
print("="*70)

for i, points in enumerate(scans[:3]):  # Analyze first 3 scans
    print(f"\nScan {i+1}:")
    print(f"  Points: {len(points)}")
    print(f"  X range: {points[:, 0].min():.2f} to {points[:, 0].max():.2f} (span: {points[:, 0].max() - points[:, 0].min():.2f}m)")
    print(f"  Y range: {points[:, 1].min():.2f} to {points[:, 1].max():.2f} (span: {points[:, 1].max() - points[:, 1].min():.2f}m)")
    print(f"  Z range: {points[:, 2].min():.2f} to {points[:, 2].max():.2f} (span: {points[:, 2].max() - points[:, 2].min():.2f}m)")

    # Check if points form a planar pattern
    # Calculate angles from origin
    distances = np.linalg.norm(points, axis=1)

    # Horizontal angle (azimuth)
    azimuth = np.arctan2(points[:, 1], points[:, 0]) * 180 / np.pi

    # Vertical angle (elevation)
    elevation = np.arctan2(points[:, 2], np.sqrt(points[:, 0]**2 + points[:, 1]**2)) * 180 / np.pi

    print(f"  Azimuth range: {azimuth.min():.1f}° to {azimuth.max():.1f}° (span: {azimuth.max() - azimuth.min():.1f}°)")
    print(f"  Elevation range: {elevation.min():.1f}° to {elevation.max():.1f}° (span: {elevation.max() - elevation.min():.1f}°)")
    print(f"  Distance range: {distances.min():.2f}m to {distances.max():.2f}m")

# Create visualization
print()
print("Creating visualization...")

fig = plt.figure(figsize=(15, 5))

for i in range(min(3, len(scans))):
    points = scans[i]

    # 3D scatter plot
    ax = fig.add_subplot(1, 3, i+1, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], cmap='viridis', s=1)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'Scan {i+1} ({len(points)} points)')

    # Equal aspect ratio
    max_range = max(
        points[:, 0].max() - points[:, 0].min(),
        points[:, 1].max() - points[:, 1].min(),
        points[:, 2].max() - points[:, 2].min()
    ) / 2.0

    mid_x = (points[:, 0].max() + points[:, 0].min()) / 2
    mid_y = (points[:, 1].max() + points[:, 1].min()) / 2
    mid_z = (points[:, 2].max() + points[:, 2].min()) / 2

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.tight_layout()
output_path = Path(__file__).parent.parent / 'data' / 'scan_structure_analysis.png'
plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f"Saved visualization to: {output_path}")

print()
print("="*70)
print("CONCLUSION")
print("="*70)
print()
print("Each LiDAR packet contains a SEGMENT of the full 3D scan.")
print("The Unitree L2 captures points using a dual-angle system:")
print("  - Horizontal (azimuth): rotating laser around vertical axis")
print("  - Vertical (elevation): multiple beams at different heights")
print()
print("For a complete 360° 3D scan, you need to:")
print("  1. Accumulate multiple packets over time")
print("  2. Keep sensor stationary OR use odometry to align scans")
print("  3. Each packet is like a 'slice' of the environment")
print("="*70)