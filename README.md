# Unitree L2 LiDAR Hiking Trail Mapping

A complete Python-based system for capturing, processing, and visualizing hiking trail data using the Unitree L2 LiDAR sensor, with interactive web-based 3D visualization.

## Project Status

✅ **Phase 1: LiDAR Data Capture and Processing** - Complete
✅ **Phase 1.5: 3D Mesh Generation & Web Visualization** - Complete
✅ **Phase 1.6: Complete SLAM with Loop Closure** - Complete
📋 **Phase 2: Backend Development** - Planned

## Features

### Data Capture & Processing
- ✅ LiDAR data capture interface for Unitree L2 sensor
- ✅ UDP-based real-time data reception
- ✅ Point cloud processing and filtering
- ✅ Statistical outlier removal
- ✅ Ground plane segmentation (RANSAC)
- ✅ Voxel-based downsampling
- ✅ Normal estimation

### 3D Mapping & Visualization
- ✅ 3D mesh generation from point clouds
- ✅ Multiple export formats (OBJ, PLY, JSON)
- ✅ Interactive web-based 3D viewer
- ✅ Height-based elevation coloring
- ✅ Multiple display modes (solid, wireframe, points)
- ✅ Camera controls and presets
- ✅ Screenshot export

### Trail Data Management
- ✅ Session-based capture organization
- ✅ Multi-format storage (NPY, NPZ, JSON, OBJ, PLY)
- ✅ Metadata tracking

### SLAM (Simultaneous Localization and Mapping)
- ✅ KISS-ICP odometry for frame-to-frame matching
- ✅ ikd-Tree for efficient incremental map storage
- ✅ Scan Context for loop closure detection
- ✅ Pose Graph Optimization for drift correction
- ✅ Handheld scanning support with motion tolerance
- ✅ IMU integration for point cloud deskewing (ORB-SLAM3 inspired)

## Project Structure

```
Lidar/
├── src/
│   ├── lidar_interface/      # Unitree L2 sensor communication
│   │   ├── unitree_l2.py     # Main sensor interface
│   │   └── data_capture.py   # Data capture sessions
│   ├── data_processing/      # Point cloud processing
│   │   └── point_cloud_processor.py
│   └── visualization/        # 3D visualization tools
│       └── visualizer.py
├── data/
│   ├── raw/                  # Raw LiDAR captures
│   └── processed/            # Processed point clouds
├── tests/                    # Unit tests
├── notebooks/                # Jupyter notebooks for exploration
├── config/                   # Configuration files
└── docs/                     # Documentation

```

## Installation

### Prerequisites

- Python 3.8 or higher
- Unitree L2 LiDAR sensor
- Git

### Setup Steps

1. Clone the repository:
```bash
git clone <repository-url>
cd Lidar
```

2. Create a virtual environment:
```bash
python -m venv venv

# On Windows:
venv\Scripts\activate

# On macOS/Linux:
source venv/bin/activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

4. Configure the LiDAR sensor:
   - Edit connection settings in `config/lidar_config.yaml` (to be created)
   - Set the correct IP address and port for your Unitree L2 sensor
   - Refer to Unitree L2 documentation for network setup

## Quick Start

### Complete Pipeline: Scan and Build 3D Map

The easiest way to get started is using the complete pipeline script:

```bash
# Scan a trail and generate 3D visualization (using synthetic data for testing)
python examples/scan_and_build_map.py my_trail --duration 60

# Or with real Unitree L2 hardware:
python examples/scan_and_build_map.py my_trail --duration 120 --real-sensor
```

This will:
1. Capture LiDAR data from the trail
2. Process and clean the point cloud
3. Generate 3D mesh
4. Export to multiple formats (OBJ, PLY, JSON)
5. Prepare for web visualization

Then view the results:

```bash
# Start the web viewer
cd web
python server.py
```

Open `http://localhost:8000` in your browser and select your trail from the dropdown.

### Manual Workflow

#### Step 1: Capturing LiDAR Data

```python
from src.lidar_interface import LiDARDataCapture

# Start a capture session
capture = LiDARDataCapture(output_dir="data/raw")

# Begin session
capture.start_session(session_name="trail_001")

# Capture frames
capture.capture_continuous(duration_seconds=60)

# End session
capture.end_session()
```

### Processing Point Cloud Data

```python
from src.data_processing import PointCloudProcessor
import numpy as np

# Load captured data
processor = PointCloudProcessor()

# Load raw points
raw_points = np.load("data/raw/trail_001/frame_000000.npy")

# Process the data
result = processor.process_trail_scan(
    raw_points,
    remove_outliers=True,
    downsample_voxel=0.05,
    segment_ground=True
)

# Save processed data
processor.save_processed_cloud(
    result['processed_points'],
    result['normals'],
    "data/processed/trail_001_processed.pcd"
)
```

### Visualizing Point Clouds

```python
from src.visualization import PointCloudVisualizer

visualizer = PointCloudVisualizer()

# Visualize processed point cloud
visualizer.visualize_point_cloud(
    result['processed_points'],
    normals=result['normals']
)

# Plot statistics
visualizer.plot_point_cloud_stats(
    result['processed_points'],
    save_path="stats.png"
)
```

## Development Roadmap

### ✅ Phase 1: LiDAR Data Capture
- [x] Project structure setup
- [x] Basic LiDAR interface
- [x] Data capture system
- [x] Point cloud processing pipeline
- [x] Visualization tools
- [x] Complete Unitree L2 UDP integration
- [x] Field testing and validation

### ✅ Phase 1.5: 3D Mesh Generation
- [x] Point cloud to mesh conversion
- [x] Web-based 3D viewer (Three.js)
- [x] Multiple export formats

### ✅ Phase 1.6: Complete SLAM System
- [x] KISS-ICP odometry
- [x] ikd-Tree incremental map storage
- [x] Scan Context loop closure
- [x] Pose Graph Optimization
- [x] Handheld scanning with IMU compensation

### 🔜 Phase 2: Backend Development
- [ ] Trail segmentation algorithms
- [ ] REST API development
- [ ] Database schema design
- [ ] Cloud storage integration

### 📋 Phase 3: Mobile App Development
- [ ] Framework selection (React Native/Flutter)
- [ ] 3D rendering on mobile
- [ ] GPS integration
- [ ] Offline map support
- [ ] UI/UX design

### 📋 Phase 4: Integration & Testing
- [ ] End-to-end testing
- [ ] Field trials
- [ ] Performance optimization
- [ ] User feedback integration

## Important Notes

### Unitree L2 Integration

⚠️ **The current implementation is a template.** You need to:

1. Obtain the official Unitree L2 SDK/API documentation
2. Update [src/lidar_interface/unitree_l2.py](src/lidar_interface/unitree_l2.py) with actual SDK calls
3. Configure network settings for your specific sensor
4. Test connection and data capture with real hardware

The placeholder code in `unitree_l2.py` generates random test data. Replace the TODO sections with actual Unitree L2 SDK implementation.

### Data Storage

LiDAR data can be very large. The system saves data in two formats:
- `.npy` files: Fast NumPy binary format for processing
- `.pcd` files: Open3D format for visualization

Configure cloud storage (Phase 2) for production use.

## Testing

Run tests:
```bash
pytest tests/
```

Run with coverage:
```bash
pytest --cov=src tests/
```

## Documentation

Detailed documentation for each module:
- [LiDAR Interface](docs/lidar_interface.md) (to be created)
- [Data Processing](docs/data_processing.md) (to be created)
- [API Reference](docs/api_reference.md) (to be created)

## Contributing

This is a personal project. If you'd like to contribute:
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Open a Pull Request

## License

[Specify your license here]

## Acknowledgments

- Unitree Robotics for the L2 LiDAR sensor
- Open3D community for point cloud processing tools
- Point Cloud Library (PCL) developers

## Contact

[Your contact information]

## Resources

- [Unitree L2 Documentation](https://www.unitree.com/)
- [Open3D Documentation](http://www.open3d.org/docs/)
- [Point Cloud Library](https://pointclouds.org/)
- [Project Documentation](unitree_lidar_project.md)