"""
Quick script to analyze the scanned room data
"""
import numpy as np

# Load processed data
data = np.load('data/processed/my_room_processed.npz')
points = data['points']

print("=" * 70)
print("SCAN DATA ANALYSIS")
print("=" * 70)
print()

print(f"Total points: {len(points)}")
print()

# Check coordinate distributions
print("Coordinate ranges:")
print(f"  X: [{points[:,0].min():.2f}, {points[:,0].max():.2f}] range={points[:,0].max()-points[:,0].min():.2f}m")
print(f"  Y: [{points[:,1].min():.2f}, {points[:,1].max():.2f}] range={points[:,1].max()-points[:,1].min():.2f}m")
print(f"  Z: [{points[:,2].min():.2f}, {points[:,2].max():.2f}] range={points[:,2].max()-points[:,2].min():.2f}m")
print()

# Count points with substantial values
non_zero_y = np.abs(points[:,1]) > 0.01
non_zero_z = np.abs(points[:,2]) > 0.01
substantial_y = np.abs(points[:,1]) > 1.0
substantial_z = np.abs(points[:,2]) > 1.0

print("Point distribution:")
print(f"  Points with |Y| > 0.01m: {non_zero_y.sum()} ({100*non_zero_y.sum()/len(points):.1f}%)")
print(f"  Points with |Z| > 0.01m: {non_zero_z.sum()} ({100*non_zero_z.sum()/len(points):.1f}%)")
print(f"  Points with |Y| > 1.0m:  {substantial_y.sum()} ({100*substantial_y.sum()/len(points):.1f}%)")
print(f"  Points with |Z| > 1.0m:  {substantial_z.sum()} ({100*substantial_z.sum()/len(points):.1f}%)")
print()

# Show sample of points with good Y values
substantial_points = points[substantial_y]
if len(substantial_points) > 0:
    print(f"Sample of {min(10, len(substantial_points))} points with |Y| > 1.0m:")
    for i, pt in enumerate(substantial_points[:10]):
        print(f"  [{i}] X={pt[0]:7.2f}, Y={pt[1]:7.2f}, Z={pt[2]:7.2f}")
else:
    print("WARNING: No points with substantial Y values!")
    print("This suggests the sensor was not moving or data parsing is incorrect")

print()
print("=" * 70)