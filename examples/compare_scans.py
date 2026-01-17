"""
Compare multiple LiDAR scans and analyze differences
Useful for trail monitoring over time or quality assessment
"""
import sys
from pathlib import Path
import numpy as np
import json
from datetime import datetime

sys.path.insert(0, str(Path(__file__).parent.parent))

print("=" * 70)
print("SCAN COMPARISON TOOL - Unitree L2 LiDAR")
print("=" * 70)
print()

# Find all available scans
data_dir = Path(__file__).parent.parent / 'data' / 'trails'
web_models_dir = Path(__file__).parent.parent / 'web' / 'models'

scans = []

# Scan data/trails directory
if data_dir.exists():
    for scan_dir in data_dir.iterdir():
        if scan_dir.is_dir():
            metadata_path = scan_dir / 'metadata.json'
            if metadata_path.exists():
                with open(metadata_path) as f:
                    metadata = json.load(f)
                    scans.append({
                        'name': scan_dir.name,
                        'path': scan_dir,
                        'metadata': metadata,
                        'source': 'trails'
                    })

# Scan web/models directory for quick scans
if web_models_dir.exists():
    for json_file in web_models_dir.glob('*.json'):
        scans.append({
            'name': json_file.stem,
            'path': json_file,
            'metadata': None,
            'source': 'web_models'
        })

if len(scans) == 0:
    print("No scans found!")
    print()
    print("Run a scan first:")
    print("  python examples/quick_scan.py")
    print("  python examples/trail_scan.py")
    sys.exit(1)

# Display available scans
print(f"Found {len(scans)} scans:")
print()

for i, scan in enumerate(scans, 1):
    print(f"{i}. {scan['name']}")
    if scan['metadata']:
        meta = scan['metadata']
        print(f"   Date: {meta.get('scan_date', 'Unknown')}")
        print(f"   Points: {meta.get('total_points_captured', 'Unknown'):,}")
        print(f"   Trail: {meta.get('trail_name', 'Unknown')}")
    print(f"   Source: {scan['source']}")
    print()

# Select scans to compare
print("=" * 70)
print()
selection = input("Enter scan numbers to compare (e.g., '1,3,5' or 'all'): ").strip()

if selection.lower() == 'all':
    selected_scans = scans
else:
    try:
        indices = [int(x.strip()) - 1 for x in selection.split(',')]
        selected_scans = [scans[i] for i in indices if 0 <= i < len(scans)]
    except (ValueError, IndexError):
        print("Invalid selection!")
        sys.exit(1)

if len(selected_scans) == 0:
    print("No scans selected!")
    sys.exit(1)

print()
print(f"Comparing {len(selected_scans)} scans...")
print()

# Comparison table
print("=" * 70)
print("COMPARISON TABLE")
print("=" * 70)
print()

# Header
print(f"{'Metric':<30} ", end='')
for scan in selected_scans:
    print(f"{scan['name'][:15]:>15} ", end='')
print()
print("-" * 70)

# Comparison metrics
metrics = []

# Load scan data and compute metrics
for scan in selected_scans:
    scan_metrics = {
        'name': scan['name'],
        'source': scan['source']
    }

    if scan['metadata']:
        meta = scan['metadata']
        scan_metrics['total_points'] = meta.get('total_points_captured', 0)
        scan_metrics['downsampled_points'] = meta.get('points_after_downsampling', 0)
        scan_metrics['vertices'] = meta.get('mesh_vertices', 0)
        scan_metrics['faces'] = meta.get('mesh_faces', 0)
        scan_metrics['duration'] = meta.get('scan_duration_seconds', 0)

        coord_range = meta.get('coordinate_range', {})
        if coord_range:
            scan_metrics['x_range'] = coord_range.get('x_max', 0) - coord_range.get('x_min', 0)
            scan_metrics['y_range'] = coord_range.get('y_max', 0) - coord_range.get('y_min', 0)
            scan_metrics['z_range'] = coord_range.get('z_max', 0) - coord_range.get('z_min', 0)
    else:
        # Try to load JSON and compute basic metrics
        try:
            with open(scan['path']) as f:
                data = json.load(f)
                vertices = np.array(data['vertices']).reshape(-1, 3)
                scan_metrics['vertices'] = len(vertices)
                scan_metrics['faces'] = len(data.get('faces', [])) // 3

                # Compute ranges
                coords_min = vertices.min(axis=0)
                coords_max = vertices.max(axis=0)
                coords_range = coords_max - coords_min
                scan_metrics['x_range'] = coords_range[0]
                scan_metrics['y_range'] = coords_range[1]
                scan_metrics['z_range'] = coords_range[2]
        except:
            pass

    metrics.append(scan_metrics)

# Print comparison rows
def print_row(label, key, formatter=lambda x: str(x)):
    print(f"{label:<30} ", end='')
    for metric in metrics:
        value = metric.get(key, 'N/A')
        formatted = formatter(value) if value != 'N/A' else 'N/A'
        print(f"{formatted:>15} ", end='')
    print()

print_row("Total Points", 'total_points', lambda x: f"{x:,}" if x else "N/A")
print_row("Downsampled Points", 'downsampled_points', lambda x: f"{x:,}" if x else "N/A")
print_row("Mesh Vertices", 'vertices', lambda x: f"{x:,}" if x else "N/A")
print_row("Mesh Faces", 'faces', lambda x: f"{x:,}" if x else "N/A")
print_row("Scan Duration (s)", 'duration', lambda x: f"{x}" if x else "N/A")
print()
print_row("X Range (m)", 'x_range', lambda x: f"{x:.2f}" if isinstance(x, (int, float)) else "N/A")
print_row("Y Range (m)", 'y_range', lambda x: f"{x:.2f}" if isinstance(x, (int, float)) else "N/A")
print_row("Z Range (m)", 'z_range', lambda x: f"{x:.2f}" if isinstance(x, (int, float)) else "N/A")

print()
print("=" * 70)

# Quality analysis
print()
print("QUALITY ANALYSIS")
print("=" * 70)
print()

for i, (scan, metric) in enumerate(zip(selected_scans, metrics), 1):
    print(f"{i}. {scan['name']}")

    # Point density
    if 'x_range' in metric and 'y_range' in metric:
        area = metric['x_range'] * metric['y_range']
        if area > 0 and 'downsampled_points' in metric and metric['downsampled_points']:
            density = metric['downsampled_points'] / area
            print(f"   Point density: {density:.0f} points/m²")

    # Mesh quality
    if 'vertices' in metric and 'faces' in metric and metric['vertices'] and metric['faces']:
        face_to_vertex_ratio = metric['faces'] / metric['vertices']
        print(f"   Mesh quality: {face_to_vertex_ratio:.2f} faces/vertex")

        if face_to_vertex_ratio < 1.5:
            print(f"   ⚠️  Low mesh density - may have gaps")
        elif face_to_vertex_ratio > 3.0:
            print(f"   ✓ Good mesh density")

    # Coverage
    if 'x_range' in metric and 'y_range' in metric and 'z_range' in metric:
        volume = metric['x_range'] * metric['y_range'] * metric['z_range']
        print(f"   Coverage volume: {volume:.2f} m³")
        print(f"   Dimensions: {metric['x_range']:.1f}m × {metric['y_range']:.1f}m × {metric['z_range']:.1f}m")

    print()

print("=" * 70)
print()
print("Recommendations:")
print()

# Find best scan
if metrics:
    best_points = max(metrics, key=lambda x: x.get('downsampled_points', 0))
    best_coverage = max(metrics, key=lambda x: x.get('y_range', 0) * x.get('x_range', 0) if 'y_range' in x and 'x_range' in x else 0)

    print(f"✓ Most detailed scan: {best_points['name']}")
    print(f"  ({best_points.get('downsampled_points', 0):,} points)")
    print()
    print(f"✓ Largest coverage: {best_coverage['name']}")
    if 'x_range' in best_coverage and 'y_range' in best_coverage:
        area = best_coverage['x_range'] * best_coverage['y_range']
        print(f"  ({area:.1f} m² area)")
    print()

print("=" * 70)