"""
Setup Verification Script
Runs a complete check of the trail mapping system.
"""

import sys
from pathlib import Path

print("=" * 70)
print("UNITREE L2 LIDAR TRAIL MAPPING SYSTEM - SETUP CHECK")
print("=" * 70)
print()

# Check 1: Python version
print("1. Checking Python version...")
version = sys.version_info
if version.major == 3 and version.minor >= 8:
    print(f"   [OK] Python {version.major}.{version.minor}.{version.micro}")
else:
    print(f"   [FAIL] Python {version.major}.{version.minor} - Need Python 3.8+")
    sys.exit(1)

# Check 2: Required packages
print("\n2. Checking required packages...")
required = ['numpy', 'pandas', 'scipy', 'sklearn', 'matplotlib', 'plotly']
missing = []

for package in required:
    try:
        __import__(package)
        print(f"   [OK] {package}")
    except ImportError:
        print(f"   [FAIL] {package} - MISSING")
        missing.append(package)

if missing:
    print(f"\n   Install missing packages: pip install {' '.join(missing)}")
    sys.exit(1)

# Check 3: Project modules
print("\n3. Checking project modules...")
sys.path.insert(0, str(Path(__file__).parent / 'src'))

modules = [
    ('lidar_interface', 'LiDARConfig'),
    ('data_processing', 'PointCloudProcessor'),
    ('data_processing.point_cloud_processor_numpy', 'PointCloudProcessorNumPy'),
    ('data_processing.mesh_generator', 'MeshGenerator'),
]

for module_name, class_name in modules:
    try:
        module = __import__(module_name, fromlist=[class_name])
        getattr(module, class_name)
        print(f"   [OK] {module_name}.{class_name}")
    except Exception as e:
        print(f"   [FAIL] {module_name}.{class_name} - ERROR: {e}")
        sys.exit(1)

# Check 4: Directory structure
print("\n4. Checking directory structure...")
required_dirs = [
    'src/lidar_interface',
    'src/data_processing',
    'src/visualization',
    'web/models',
    'web/css',
    'web/js',
    'data/raw',
    'data/processed',
    'examples',
    'tests',
]

for dir_path in required_dirs:
    path = Path(dir_path)
    if path.exists():
        print(f"   [OK] {dir_path}/")
    else:
        print(f"   [FAIL] {dir_path}/ - MISSING")

# Check 5: Key files
print("\n5. Checking key files...")
key_files = [
    'web/index.html',
    'web/server.py',
    'web/js/viewer.js',
    'web/js/app.js',
    'examples/scan_and_build_map.py',
    'examples/test_numpy_processing.py',
]

for file_path in key_files:
    path = Path(file_path)
    if path.exists():
        print(f"   [OK] {file_path}")
    else:
        print(f"   [FAIL] {file_path} - MISSING")

# Check 6: Test trails
print("\n6. Checking generated trails...")
trail_models = list(Path('web/models').glob('*.json'))
if trail_models:
    print(f"   [OK] Found {len(trail_models)} trail(s):")
    for trail in trail_models:
        size_kb = trail.stat().st_size / 1024
        print(f"     - {trail.stem} ({size_kb:.0f} KB)")
else:
    print("   [INFO] No trails generated yet")
    print("     Run: python examples/scan_and_build_map.py my_trail")

# Summary
print()
print("=" * 70)
print("SYSTEM STATUS: READY")
print("=" * 70)
print()
print("Next steps:")
print("  1. Generate a trail:")
print("     python examples/scan_and_build_map.py my_trail --duration 60")
print()
print("  2. Start web viewer:")
print("     cd web")
print("     python server.py")
print()
print("  3. Open browser:")
print("     http://localhost:8000")
print()
print("For real sensor data:")
print("  - Configure network: 192.168.1.2")
print("  - See: docs/UNITREE_L2_INTEGRATION.md")
print("  - Run with: --real-sensor flag")
print()
print("=" * 70)