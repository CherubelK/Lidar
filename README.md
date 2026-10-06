# Unitree L2 LiDAR SLAM Toolkit

A Python toolkit for the Unitree 4D LiDAR L2: raw UDP capture, point-cloud processing, a complete SLAM pipeline with loop closure, and a browser-based 3D viewer. Built and validated on real hardware.

- **Capture** the L2's UDP packet stream directly, with no vendor SDK, and convert it to metre-scale 3D points.
- **Map** with KISS-ICP odometry, an incremental k-d tree, Scan Context loop detection, and pose-graph optimisation, including handheld scanning with IMU deskewing.
- **Inspect** results in a Three.js viewer that supports live streaming, scan comparison, change detection, and floor-plan generation.

## Status

| Area | State |
|---|---|
| Sensor capture and 3D transform | Validated on hardware: ~287 points per packet, 586k points in a 10-second room scan |
| Point-cloud pipeline | Outlier removal, voxel downsampling, RANSAC ground segmentation, normal estimation, Delaunay meshing (71k faces on a room scan) |
| SLAM with loop closure | Working; a 75-scan handheld test closed 4 loops |
| Web viewer | Working: viewer, live view, scan control, walkthrough, compare, floor plan |
| Change detection, occupancy, material classification | Implemented in `src/data_processing/`, exposed through the web server |
| Multi-device mesh networking | Prototype in `src/networking/` |
| Next phase | Pivoted from trail mapping to recurring warehouse inventory scans. See [PROGRESS.md](PROGRESS.md) for the market analysis and decision |

## How the SLAM pipeline fits together

```
L2 UDP packets ──► parse + calibrate ──► point cloud (N×3, metres)
                                              │
                                              ▼
                                   KISS-ICP odometry ──► pose (4×4)
                                              │              │
                            Scan Context descriptor     pose graph (odometry edge)
                                              │              │
                                  loop detected? ──yes──► ICP verify ──► loop edge
                                              │                               │
                                              ▼                               ▼
                                   ikd-Tree global map ◄──── pose-graph optimisation
                                              │
                                              ▼
                              NPY / PLY / OBJ / JSON  ──►  web viewer
```

Full component descriptions and tuning parameters are in [docs/COMPLETE_SLAM.md](docs/COMPLETE_SLAM.md).

## Quick start

### Requirements
- Python 3.11 is recommended. Python 3.13 works; Open3D does not yet support it, so the NumPy processing path is used automatically.
- A Unitree L2 on the same network for live capture. Everything else runs on the committed sample scans.

### Install

```bash
git clone https://github.com/CherubelK/Lidar.git
cd Lidar
python -m venv venv
source venv/bin/activate          # Windows: venv\Scripts\activate
pip install -r requirements.txt
python setup_check.py             # verifies the environment
```

### Connect the sensor

The L2 streams UDP from `192.168.1.62:6101` to a host at `192.168.1.2:6201` by default. Give your machine the static IP `192.168.1.2` on the interface wired to the sensor, allow inbound UDP 6201 through the firewall, then:

```bash
python examples/test_sensor_connection.py
```

Defaults live in `UnitreeL2Config` in [src/lidar_interface/unitree_l2_udp.py](src/lidar_interface/unitree_l2_udp.py). Packet format and calibration details are in [docs/UNITREE_L2_INTEGRATION.md](docs/UNITREE_L2_INTEGRATION.md).

### Scan

```bash
# 10-second static room scan, exported for the web viewer
python examples/quick_scan.py

# Full SLAM with loop closure (move slowly, return to the start to close the loop)
python examples/complete_slam_scan.py --duration 120 --name my_room

# Handheld SLAM with IMU deskewing
python examples/handheld_slam_scan.py --duration 60 --name walkthrough
```

Outputs land in `data/<mode>/<name_timestamp>/` as NPY, PLY, OBJ, JSON, plus poses, trajectory, and metadata.

### View

```bash
cd web
python server.py                  # serves on http://localhost:8000 and opens a browser
```

| Page | Purpose |
|---|---|
| `viewer.html` | Rotate, pan, zoom; solid, wireframe, and point modes; height colouring; camera presets; screenshots |
| `live.html` | Live point stream from the sensor |
| `scan.html` | Start and stop scans from the browser |
| `walkthrough.html` | First-person walkthrough of a map |
| `compare.html` | Diff two scans with change detection |
| `floorplan.html` | 2D floor plan from a scan |

## Repository layout

```
src/
  lidar_interface/     UDP packet parser, calibration, capture sessions
  data_processing/     point-cloud ops, KISS-ICP, ikd-Tree, Scan Context, pose graph,
                       IMU integration, Point-LIO, change detection, occupancy, materials
  visualization/       matplotlib plots and statistics
  networking/          multi-device mesh prototype, map merging, encryption
web/                   Three.js viewer and the Python dev server
examples/              runnable scripts for every capture and SLAM mode, each with a README
docs/                  SLAM internals, hardware integration, system wiring, GPS notes
tests/                 pytest suite (synthetic data, no hardware needed)
config/                lidar_config.example.yaml
data/                  sample scans from real sessions (room scans, handheld SLAM runs)
```

## Documentation

- [GETTING_STARTED.md](GETTING_STARTED.md): step-by-step setup
- [docs/COMPLETE_SLAM.md](docs/COMPLETE_SLAM.md): odometry, mapping, loop closure, optimisation
- [docs/UNITREE_L2_INTEGRATION.md](docs/UNITREE_L2_INTEGRATION.md): packet format and coordinate transform
- [docs/SYSTEM_INTEGRATION.md](docs/SYSTEM_INTEGRATION.md): L2 + Raspberry Pi + RTK GPS wiring and power
- [docs/MULTI_DEVICE_MESH.md](docs/MULTI_DEVICE_MESH.md): multi-sensor networking design
- [docs/WHY_GPS_FAILS_INDOORS.md](docs/WHY_GPS_FAILS_INDOORS.md): why the indoor path uses SLAM rather than GNSS
- [docs/QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md): command cheat sheet
- `examples/README_*.md`: one guide per scanning mode (scanning, ICP odometry, KISS-ICP, GPS setup, positioning options, multi-device)
- [PROGRESS.md](PROGRESS.md): build log, validation results, and the Phase 2 decision

## Tests

```bash
pytest tests/ -v
```

Covers the processing pipeline, meshing, and point-cloud merging using synthetic data.

## Acknowledgements

Algorithms follow the published KISS-ICP, ikd-Tree, Scan Context, and Point-LIO papers. Visualisation uses Three.js. Thanks to Unitree Robotics for the L2 and to the Open3D community.

## Author

Cherubel Kefyalew · [github.com/CherubelK](https://github.com/CherubelK)
