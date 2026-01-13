"""
Comprehensive diagnostic to find the issue
"""
import numpy as np
import json

print("=" * 70)
print("DIAGNOSTIC: Finding the Issue")
print("=" * 70)
print()

# 1. Check raw processed data
print("1. PROCESSED POINT CLOUD DATA")
print("-" * 70)
data = np.load('data/processed/my_room_processed.npz')
points = data['points']

print(f"Total points: {len(points)}")
print(f"X range: [{points[:,0].min():.2f}, {points[:,0].max():.2f}] = {points[:,0].max()-points[:,0].min():.2f}m")
print(f"Y range: [{points[:,1].min():.2f}, {points[:,1].max():.2f}] = {points[:,1].max()-points[:,1].min():.2f}m")
print(f"Z range: [{points[:,2].min():.2f}, {points[:,2].max():.2f}] = {points[:,2].max()-points[:,2].min():.2f}m")

# Check std dev to see spread
print(f"\nStandard deviations:")
print(f"  X: {points[:,0].std():.2f}m")
print(f"  Y: {points[:,1].std():.2f}m")
print(f"  Z: {points[:,2].std():.2f}m")

print()

# 2. Check mesh JSON
print("2. MESH JSON DATA")
print("-" * 70)
with open('web/models/my_room.json', 'r') as f:
    mesh_data = json.load(f)

positions = np.array(mesh_data['data']['attributes']['position']['array']).reshape(-1, 3)
indices = np.array(mesh_data['data']['index']['array'])

print(f"Vertices in JSON: {len(positions)}")
print(f"Faces in JSON: {len(indices)//3}")
print(f"JSON X range: [{positions[:,0].min():.2f}, {positions[:,0].max():.2f}]")
print(f"JSON Y range: [{positions[:,1].min():.2f}, {positions[:,1].max():.2f}]")
print(f"JSON Z range: [{positions[:,2].min():.2f}, {positions[:,2].max():.2f}]")

# Check if positions match processed points
if np.allclose(positions, points, atol=1e-5):
    print("\n✓ Mesh vertices match processed points exactly")
else:
    print("\n✗ WARNING: Mesh vertices differ from processed points!")

print()

# 3. Check face validity
print("3. MESH FACE VALIDITY")
print("-" * 70)
num_faces = len(indices) // 3
faces = indices.reshape(-1, 3)

# Check for degenerate faces (faces with duplicate vertices)
degenerate = 0
for i, face in enumerate(faces[:100]):  # Check first 100
    if face[0] == face[1] or face[1] == face[2] or face[0] == face[2]:
        degenerate += 1

print(f"Degenerate faces (first 100): {degenerate}")

# Check face sizes
face_areas = []
for i in range(min(1000, num_faces)):
    v0, v1, v2 = positions[faces[i]]
    # Calculate area using cross product
    area = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0))
    face_areas.append(area)

face_areas = np.array(face_areas)
print(f"Face areas (first 1000):")
print(f"  Min: {face_areas.min():.6f} m²")
print(f"  Max: {face_areas.max():.6f} m²")
print(f"  Mean: {face_areas.mean():.6f} m²")
print(f"  Median: {np.median(face_areas):.6f} m²")

# Check for very large faces that might indicate bad triangulation
large_faces = np.sum(face_areas > 10.0)
print(f"  Very large faces (>10m²): {large_faces}")

print()

# 4. Data distribution analysis
print("4. DATA DISTRIBUTION ANALYSIS")
print("-" * 70)

# Check if data is roughly planar
from sklearn.decomposition import PCA
pca = PCA(n_components=3)
pca.fit(points)

print("Principal components (explained variance):")
for i, var in enumerate(pca.explained_variance_):
    print(f"  PC{i+1}: {var:.2f} ({pca.explained_variance_ratio_[i]*100:.1f}%)")

if pca.explained_variance_ratio_[2] < 0.01:
    print("\n⚠ WARNING: Data is very planar (3rd component < 1%)")
    print("  This suggests the scan was mostly in 2D, not a full 3D room scan")

print()

# 5. Recommendation
print("=" * 70)
print("DIAGNOSIS")
print("=" * 70)

if pca.explained_variance_ratio_[2] < 0.05:
    print("❌ ISSUE IDENTIFIED: Data is too planar")
    print()
    print("The LiDAR scan captured mostly 2D data (like scanning a wall)")
    print("instead of a full 3D room. This happens when:")
    print("  1. Sensor was stationary during scan")
    print("  2. Sensor only rotated in one plane")
    print("  3. Packet parsing is extracting wrong coordinates")
    print()
    print("SOLUTION: Rescan with sensor moving around the room,")
    print("or check if packet parsing swapped Y/Z coordinates")
elif len(np.unique(positions, axis=0)) < len(positions) * 0.5:
    print("❌ ISSUE: Too many duplicate vertices")
    print("This suggests outlier removal or downsampling was too aggressive")
elif large_faces > len(face_areas) * 0.1:
    print("❌ ISSUE: Mesh has too many large faces")
    print("Delaunay triangulation created invalid mesh")
    print("Try using ball-pivoting instead")
else:
    print("✓ Data looks reasonable")
    print("Issue is likely in the web viewer rendering")

print("=" * 70)