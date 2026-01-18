# LiDAR Scanning Guide

## Understanding the Unitree L2

The Unitree L2 is a **stationary 3D LiDAR** designed for:
- Fixed mounting positions
- 360° environmental scanning
- Static mapping applications

It is **NOT designed for** handheld mobile scanning while walking.

## Why Mobile Scanning Doesn't Work Properly

When you walk around with the LiDAR:

1. **IMU provides**: Rotation/orientation (which way sensor is pointing)
2. **IMU does NOT provide**: Position/translation (where sensor has moved to)

Without knowing the sensor's position, all scans get overlaid at the origin, creating the scattered fragmented mesh you're seeing.

## Solutions for Room Mapping

### Solution 1: Stationary Multi-Position Scanning (RECOMMENDED)
**Best approach for the Unitree L2:**

1. Place sensor in corner of room
2. Run 10-second scan (captures ~280k points)
3. Move sensor to different corner
4. Run another 10-second scan
5. Manually align scans in software (CloudCompare, MeshLab, etc.)

**Advantages:**
- Works with equipment as-is
- High quality scans
- No complex algorithms needed

**Scripts to use:**
- `quick_scan.py` - For each stationary position

### Solution 2: ICP-based Scan Matching (Experimental)
Use point cloud registration to estimate sensor movement.

**Limitations:**
- Requires good overlap between consecutive scans
- Accumulates error over time (drift)
- Computationally expensive
- Works best in feature-rich environments

**Not yet implemented** - Would require Open3D or similar library.

### Solution 3: External Tracking
Add additional sensors to track position:

**Options:**
- Visual SLAM camera (RealSense, ZED, etc.)
- GPS (outdoor only, low accuracy)
- Motion capture system (expensive)
- Wheel odometry (if mounted on robot)

## Current Scanning Scripts

### For Stationary Use:
- **`quick_scan.py`** - 10-second scan, best for fixed positions
- **`scan_room_real.py`** - Original room scanning script

### For Mobile Use (Limited):
- **`trail_scan.py`** - 60-second trail scan (overlapping points)
- **`slam_scan.py`** - Voxel filtering to remove duplicates
- **`imu_slam_scan.py`** - Uses IMU for rotation correction

**Note:** Mobile scripts will produce scattered/fragmented results because they cannot track position.

## Recommended Workflow

**For a complete room scan:**

1. Choose 4-6 positions around the room (corners, center, etc.)
2. For each position:
   ```bash
   python examples/quick_scan.py
   ```
3. Manually name each scan (e.g., "corner1", "corner2", etc.)
4. Use external software to align and merge:
   - **CloudCompare** (free): Manual alignment with ICP refinement
   - **MeshLab** (free): Point cloud alignment
   - **Autodesk ReCap** (paid): Automatic alignment

## Future Improvements

To enable true mobile scanning, we would need to implement:

1. **LiDAR Odometry** (LOAM/LEGO-LOAM algorithms)
   - Track movement by matching consecutive scans
   - Requires Open3D or PCL library
   - Complex implementation

2. **Visual-Inertial SLAM**
   - Add camera for visual tracking
   - Combine with IMU data
   - Requires ROS or custom SLAM implementation

3. **Graph-based SLAM**
   - Build pose graph of sensor positions
   - Optimize entire trajectory
   - Requires GTSAM or g2o library

These are significant engineering projects beyond the current scope.

## Conclusion

**For best results with the Unitree L2:**
- Use it as a **stationary scanner**
- Take multiple scans from different positions
- Manually align them in post-processing software

This approach will give you much better results than trying to scan while walking around.