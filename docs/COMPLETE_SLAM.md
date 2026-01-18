# Complete SLAM System with Loop Closure

## Overview

This document describes the complete SLAM (Simultaneous Localization and Mapping) system with loop closure detection and pose graph optimization.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Complete SLAM Pipeline                            │
│                                                                          │
│  ┌──────────────┐    ┌───────────────┐    ┌──────────────────────────┐  │
│  │  LiDAR Scan  │───►│  KISS-ICP     │───►│  Pose Estimate           │  │
│  │  (Nx3 points)│    │  Odometry     │    │  (4x4 transform)         │  │
│  └──────────────┘    └───────────────┘    └────────────┬─────────────┘  │
│         │                                              │                 │
│         │                                              ▼                 │
│         │                                   ┌─────────────────────────┐  │
│         │                                   │  Pose Graph             │  │
│         │                                   │  (add odometry edge)    │  │
│         │                                   └────────────┬────────────┘  │
│         │                                                │               │
│         ▼                                                │               │
│  ┌──────────────┐                                        │               │
│  │  Scan        │    ┌───────────────┐                   │               │
│  │  Context     │───►│  Loop         │    Yes            │               │
│  │  Descriptor  │    │  Detected?    │────┬──────────────┤               │
│  └──────────────┘    └───────────────┘    │              │               │
│                             │ No          │              │               │
│                             ▼             ▼              ▼               │
│                      ┌────────────┐  ┌─────────┐  ┌─────────────────┐   │
│                      │  Continue  │  │  ICP    │  │  Add Loop       │   │
│                      │  Scanning  │  │  Verify │  │  Constraint     │   │
│                      └────────────┘  └────┬────┘  └────────┬────────┘   │
│                                           │                │            │
│         ┌──────────────┐                  │                ▼            │
│         │  ikd-Tree    │◄─────────────────┴───────►┌──────────────────┐ │
│         │  Global Map  │                           │  Pose Graph      │ │
│         │              │◄──────────────────────────│  Optimization    │ │
│         └──────────────┘    Corrected Poses        └──────────────────┘ │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

## Components

### 1. KISS-ICP Odometry

**File**: `src/data_processing/kiss_icp_odometry.py`

Frame-to-frame motion estimation using point cloud matching.

**Key Features**:
- Adaptive voxel size
- Motion compensation
- Real-time performance (~100ms per scan)

**Parameters**:
| Parameter | Default | Description |
|-----------|---------|-------------|
| `voxel_size` | 0.05m | Downsampling resolution |
| `max_range` | 20.0m | Maximum point distance |

### 2. ikd-Tree Map

**File**: `src/data_processing/ikd_tree.py`

Incremental KD-Tree for efficient point cloud storage and queries.

**Key Features**:
- O(log n) insertion and deletion
- Range queries for local map extraction
- Automatic rebalancing

**Operations**:
```python
from src.data_processing import IKDTree

tree = IKDTree(downsample_size=0.05)
tree.insert_points(points)                    # Add points
nearest, dist = tree.nearest_neighbor(query)  # Find nearest
neighbors = tree.k_nearest(query, k=5)        # K-nearest
in_range = tree.range_search(center, radius)  # Range query
tree.delete_by_box(box_min, box_max)          # Box deletion
```

### 3. Scan Context (Loop Closure)

**File**: `src/data_processing/scan_context.py`

Place recognition using 2D descriptors from 3D point clouds.

**How it Works**:
1. Project point cloud to bird's eye view
2. Divide into polar grid (rings × sectors)
3. Encode max height in each cell
4. Compare descriptors for loop detection

```
        Sector (angle)
        0   1   2   3   ...   59
      ┌───┬───┬───┬───┬───┬───┐
    0 │ h │ h │ h │ h │...│ h │  Ring 0 (closest)
      ├───┼───┼───┼───┼───┼───┤
    1 │ h │ h │ h │ h │...│ h │  Ring 1
      ├───┼───┼───┼───┼───┼───┤
  ... │...│...│...│...│...│...│
      ├───┼───┼───┼───┼───┼───┤
   19 │ h │ h │ h │ h │...│ h │  Ring 19 (farthest)
      └───┴───┴───┴───┴───┴───┘

   h = max height of points in cell
```

**Parameters**:
| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_sectors` | 60 | Angular resolution |
| `num_rings` | 20 | Radial resolution |
| `max_range` | 80.0m | Maximum distance |
| `sc_dist_threshold` | 0.2 | Loop detection threshold |

### 4. Pose Graph Optimization

**File**: `src/data_processing/pose_graph.py`

Graph-based optimization to correct drift when loops are detected.

**Node**: Robot pose at time t (4x4 transformation matrix)
**Edge**: Constraint between poses (relative transformation)

**Edge Types**:
- **Odometry edges**: Sequential poses (high confidence)
- **Loop edges**: Non-sequential matches (from loop closure)

**Optimization**:
Uses Gauss-Newton to minimize error:
```
minimize Σ (e_ij)^T Ω_ij (e_ij)

where:
  e_ij = error between measured and estimated relative pose
  Ω_ij = information matrix (inverse covariance)
```

### 5. Complete SLAM System

**File**: `src/data_processing/complete_slam.py`

Integrates all components into a unified system.

**Usage**:
```python
from src.data_processing import CompleteSLAM, SLAMConfig

config = SLAMConfig(
    voxel_size=0.05,
    max_range=20.0,
    loop_closure_enabled=True
)

slam = CompleteSLAM(config)

for scan in scans:
    transformed, pose, info = slam.process_scan(scan)

    if info['loop_detected']:
        print(f"Loop closure: matched scan {info['loop_idx']}")

# Get results
map_points = slam.get_map_points()
trajectory = slam.get_trajectory()
slam.save("output/", prefix="my_scan")
```

## Configuration

### SLAMConfig Options

```python
@dataclass
class SLAMConfig:
    # Odometry
    voxel_size: float = 0.05          # 5cm voxels
    max_range: float = 20.0           # 20m max range

    # Map
    map_voxel_size: float = 0.1       # 10cm map resolution
    local_map_radius: float = 50.0    # 50m local map

    # Loop closure
    loop_closure_enabled: bool = True
    loop_min_gap: int = 50            # Min 50 scans between matches
    loop_dist_threshold: float = 0.2  # SC distance threshold

    # Optimization
    optimize_every_n_loops: int = 1   # Optimize after each loop

    # Output
    save_trajectory: bool = True
    save_map: bool = True
```

### Recommended Settings

**Indoor (small rooms)**:
```python
config = SLAMConfig(
    voxel_size=0.02,      # 2cm (more detail)
    max_range=10.0,       # 10m range
    loop_min_gap=30,      # Faster loop detection
)
```

**Indoor (large building)**:
```python
config = SLAMConfig(
    voxel_size=0.05,      # 5cm
    max_range=20.0,       # 20m range
    loop_min_gap=50,
)
```

**Outdoor**:
```python
config = SLAMConfig(
    voxel_size=0.1,       # 10cm (handle scale)
    max_range=50.0,       # 50m range
    loop_min_gap=100,     # Larger environments
)
```

## Algorithm Details

### Loop Closure Detection

**Step 1: Create Scan Context**
```
For each point (x, y, z):
  range = sqrt(x² + y²)
  angle = atan2(y, x)
  ring_idx = range / ring_width
  sector_idx = (angle + π) / sector_width

  descriptor[ring_idx][sector_idx] = max(z, descriptor[ring_idx][sector_idx])
```

**Step 2: Create Ring Key (rotation invariant)**
```
ring_key[i] = mean(descriptor[i][:])  # Mean of each ring
```

**Step 3: Find Candidates**
```
For each historical scan j:
  if |current_idx - j| < min_gap:
    continue  # Too recent

  dist = ||ring_key_current - ring_key_j||
  candidates.append((j, dist))

Select top-K candidates by ring key distance
```

**Step 4: Verify with Full Descriptor**
```
For each candidate j:
  # Try all rotations
  for shift in range(num_sectors):
    sc_shifted = roll(sc_j, shift)
    dist = 1 - cosine_similarity(sc_current, sc_shifted)

    if dist < best_dist:
      best_dist = dist
      best_yaw = shift * (2π / num_sectors)

if best_dist < threshold:
  LOOP DETECTED with yaw offset
```

### Pose Graph Optimization

**Gauss-Newton Iteration**:
```
1. Linearize error function around current estimate
2. Build sparse Hessian matrix H and gradient b
3. Solve H Δx = -b
4. Update poses: x ← x + Δx
5. Repeat until convergence
```

**Error Function** (for edge i→j):
```
e_ij = log(T_measurement⁻¹ · T_i⁻¹ · T_j)

where:
  T_i = pose of node i
  T_j = pose of node j
  T_measurement = measured relative transformation
  log() = SE(3) logarithm map
```

## Performance

### Timing (Raspberry Pi 4)

| Operation | Time |
|-----------|------|
| KISS-ICP registration | ~100ms |
| Scan Context creation | ~10ms |
| Loop detection (search) | ~5ms |
| ICP verification | ~50ms |
| Pose graph optimization | ~200ms (per loop) |

### Memory Usage

| Component | Memory |
|-----------|--------|
| ikd-Tree (1M points) | ~50 MB |
| Scan Context DB (1000 scans) | ~5 MB |
| Pose Graph (1000 nodes) | ~1 MB |

## Best Practices

### For Best Loop Closure

1. **Move slowly**: 10-20cm per step, wait 2-3 seconds
2. **Complete the loop**: Return to starting position
3. **Ensure overlap**: Each scan should overlap 60-70% with previous
4. **Consistent environment**: Avoid changing scenes (moving objects)

### Movement Pattern

```
Recommended: Perimeter + Return

    Start/End
        ↓
    ┌───────────────┐
    │               │
    │   ┌───────┐   │
    │   │       │   │
    │   │       │   │
    │   │       │   │
    │   └───────┘   │
    │               │
    └───────────────┘
         Walk perimeter, then return to start
```

### Troubleshooting

**Problem: No loop closures detected**
- Solution: Walk a complete loop (return to start)
- Solution: Move slower (better Scan Context descriptors)
- Solution: Lower `loop_dist_threshold` (e.g., 0.3)

**Problem: False loop closures**
- Solution: Increase `loop_min_gap` (e.g., 100)
- Solution: Increase `loop_dist_threshold` (e.g., 0.15)
- Solution: Environments with repeated structure are challenging

**Problem: Map still has drift after optimization**
- Solution: Add more loop closure constraints
- Solution: Walk multiple loops through the same areas
- Solution: Check ICP verification quality

## Output Files

After running `CompleteSLAM.save()`:

```
output_dir/
├── slam_scan_map.npy          # Map points (Nx3)
├── slam_scan_map.ply          # Map as PLY mesh
├── slam_scan_trajectory.npy   # Trajectory (Nx3)
├── slam_scan_poses.npy        # All poses (Nx4x4)
├── slam_scan_scan_contexts.npz # Scan Context database
├── slam_scan_metadata.json    # Config and statistics
└── slam_scan_loop_info.json   # Loop closure details
```

## Comparison with Point-LIO

| Feature | Point-LIO | Our Implementation |
|---------|-----------|-------------------|
| **Odometry** | Point-by-point iEKF | KISS-ICP (scan-to-scan) |
| **IMU Fusion** | Yes (tightly coupled) | No (LiDAR only) |
| **Map Storage** | ikd-Tree | ikd-Tree |
| **Loop Closure** | No | Yes (Scan Context) |
| **Pose Graph** | No | Yes |
| **Language** | C++ | Python |
| **ROS Required** | Yes (ROS2) | No |
| **Performance** | Faster | Easier to modify |

Our system adds loop closure and pose graph optimization, which Point-LIO lacks. However, Point-LIO has tighter IMU integration which helps under aggressive motion.

## References

1. **KISS-ICP**: Vizzo et al., "KISS-ICP: In Defense of Point-to-Point ICP"
2. **ikd-Tree**: Cai et al., "ikd-Tree: An Incremental K-D Tree for Robotic Applications"
3. **Scan Context**: Kim & Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition"
4. **Point-LIO**: He et al., "Point-LIO: Robust High-Bandwidth LiDAR-Inertial Odometry"
5. **g2o**: Kümmerle et al., "g2o: A General Framework for Graph Optimization"