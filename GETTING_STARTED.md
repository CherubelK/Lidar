# Getting Started with Unitree L2 LiDAR Trail Mapping

This guide will help you set up and start using the Unitree L2 LiDAR trail mapping system.

## Step 1: Environment Setup

### Install Python

Make sure you have Python 3.8 or higher installed:

```bash
python --version
```

### Create Virtual Environment

```bash
# Create virtual environment
python -m venv venv

# Activate it
# Windows:
venv\Scripts\activate

# macOS/Linux:
source venv/bin/activate
```

### Install Dependencies

```bash
pip install -r requirements.txt
```

This will install:
- NumPy, Pandas for data processing
- Open3D for point cloud processing and visualization
- Matplotlib, Plotly for plotting
- And other required libraries

## Step 2: Configure the Unitree L2 Sensor

### Hardware Setup

1. Connect the Unitree L2 LiDAR sensor to your computer
2. Note the sensor's IP address and port (refer to Unitree documentation)
3. Ensure network connectivity between your computer and the sensor

### Software Configuration

1. Copy the example configuration:
```bash
cp config/lidar_config.example.yaml config/lidar_config.yaml
```

2. Edit `config/lidar_config.yaml` with your sensor's settings:
```yaml
lidar:
  ip_address: "192.168.1.1"  # Your sensor's IP
  port: 2368                  # Your sensor's port
```

## Step 3: Integrate Unitree L2 SDK

⚠️ **Important**: The current code is a template. You need to integrate the actual Unitree L2 SDK.

1. Download the Unitree L2 SDK from the official website
2. Read the SDK documentation
3. Update [src/lidar_interface/unitree_l2.py](src/lidar_interface/unitree_l2.py):
   - Replace the `connect()` method with actual SDK connection code
   - Replace the `get_point_cloud()` method with actual data capture code
   - Implement proper streaming based on SDK documentation

Look for `TODO` comments in the code for specific areas that need implementation.

## Step 4: Test the Installation

### Run Tests

Verify that the processing pipeline works:

```bash
pytest tests/ -v
```

All tests should pass (they use synthetic data).

### Test Visualization

Run the example visualization script:

```bash
python examples/process_and_visualize.py
```

This will:
- Generate synthetic trail data
- Process the point cloud
- Display 3D visualizations
- Save statistics plots

You should see:
- An interactive 3D viewer window
- Processing statistics in the console
- A saved plot file `trail_stats.png`

## Step 5: Capture Your First Trail

Once you've integrated the Unitree L2 SDK, capture real data:

### Basic Capture

```bash
python examples/basic_capture.py
```

This will:
- Connect to the LiDAR sensor
- Capture data for 10 seconds
- Save frames to `data/raw/test_trail/`

### Custom Capture Session

```python
from src.lidar_interface import LiDARDataCapture, LiDARConfig

# Configure sensor
config = LiDARConfig(
    ip_address="192.168.1.1",  # Your sensor IP
    port=2368,
    frame_rate=10
)

# Create capture session
with LiDARDataCapture(output_dir="data/raw", lidar_config=config) as capture:
    capture.start_session("my_trail_name")

    # Option 1: Capture for specific duration
    capture.capture_continuous(duration_seconds=60)

    # Option 2: Capture specific number of frames
    # capture.capture_continuous(num_frames=100)

    # Option 3: Capture individual frames
    # for i in range(50):
    #     point_cloud = capture.capture_frame()
```

## Step 6: Process Captured Data

### Load and Process

```python
from src.data_processing import PointCloudProcessor
from src.lidar_interface import LiDARDataCapture
import numpy as np

# Load captured session
capture = LiDARDataCapture(output_dir="data/raw")
frames = capture.load_session_frames("my_trail_name")

# Process first frame
processor = PointCloudProcessor()
result = processor.process_trail_scan(
    frames[0],
    remove_outliers=True,
    downsample_voxel=0.05,
    segment_ground=True
)

# Save processed data
processor.save_processed_cloud(
    result['processed_points'],
    result['normals'],
    "data/processed/my_trail_processed.pcd"
)
```

### Visualize Results

```python
from src.visualization import PointCloudVisualizer

viz = PointCloudVisualizer()

# Show processed trail
viz.visualize_point_cloud(
    result['processed_points'],
    normals=result['normals']
)

# Compare ground vs features
viz.visualize_multiple_clouds(
    [result['ground_points'], result['non_ground_points']],
    labels=['Ground', 'Features'],
    colors=[[0.7, 0.7, 0.7], [0.2, 0.8, 0.2]]
)

# Plot statistics
viz.plot_point_cloud_stats(
    result['processed_points'],
    save_path="my_trail_stats.png"
)
```

## Step 7: Next Steps

### Phase 1 Completion Checklist

- [ ] Successfully connect to Unitree L2 sensor
- [ ] Capture real LiDAR data from a test area
- [ ] Process and clean the data
- [ ] Visualize point clouds in 3D
- [ ] Capture a complete hiking trail
- [ ] Validate data quality and coverage

### Moving to Phase 2

Once Phase 1 is complete, you can move to:
- Converting point clouds to 3D meshes
- Building the backend API
- Setting up database and cloud storage

See [unitree_lidar_project.md](unitree_lidar_project.md) for the complete roadmap.

## Troubleshooting

### Common Issues

**Cannot connect to sensor:**
- Check network connectivity
- Verify IP address and port in configuration
- Ensure sensor is powered on
- Check firewall settings

**Import errors:**
- Make sure virtual environment is activated
- Reinstall dependencies: `pip install -r requirements.txt`
- Check Python version: `python --version` (should be 3.8+)

**Visualization not working:**
- Open3D may require specific system libraries
- On Linux: `sudo apt-get install libgl1-mesa-glx`
- On macOS: Install XQuartz if using remote display

**Low quality point clouds:**
- Adjust sensor parameters in configuration
- Check for sensor obstructions
- Increase frame rate or capture duration
- Verify proper sensor calibration

### Getting Help

1. Check the [README.md](README.md) for general information
2. Review example scripts in the `examples/` folder
3. Look at test files in `tests/` for usage patterns
4. Read the Unitree L2 official documentation
5. Open an issue on the project repository

## Best Practices

### Data Capture

- Start with short test captures (10-30 seconds)
- Walk at steady pace for consistent coverage
- Keep sensor level and stable
- Avoid rapid movements
- Capture in good visibility conditions
- Note environmental conditions in session names

### Data Processing

- Always inspect raw data before processing
- Save both raw and processed versions
- Use version control for processing parameters
- Document any manual adjustments
- Keep backup of original captures

### Performance

- Process data in batches for large trails
- Use downsampling for quick previews
- Save processed data in efficient formats
- Clean up old test data regularly

## Resources

- **Unitree L2 Documentation**: https://www.unitree.com/
- **Open3D Tutorials**: http://www.open3d.org/docs/
- **Point Cloud Library**: https://pointclouds.org/
- **Project Roadmap**: [unitree_lidar_project.md](unitree_lidar_project.md)

Happy mapping! 🗺️