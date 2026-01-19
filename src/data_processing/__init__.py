"""
Data Processing Module
Point cloud processing, filtering, and transformation utilities.

Includes:
- Point cloud processing and filtering
- KISS-ICP odometry
- ikd-Tree for efficient map storage
- Scan Context for loop closure detection
- Pose Graph Optimization
- Complete SLAM system
- Point-LIO: Tightly-coupled LiDAR-Inertial Odometry
"""

try:
    from .point_cloud_processor import PointCloudProcessor
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    # Fall back to NumPy-based processor
    from .point_cloud_processor_numpy import PointCloudProcessorNumPy as PointCloudProcessor

# SLAM components - import with error handling
try:
    from .kiss_icp_odometry import KISSICPOdometry
except ImportError as e:
    KISSICPOdometry = None
    print(f"Warning: KISS-ICP not available: {e}")

from .ikd_tree import IKDTree
from .scan_context import ScanContext, ScanContextConfig
from .pose_graph import PoseGraph
from .imu_integration import IMUIntegration, IMUPreintegration
from .imu_position import (
    IMUPositionEstimator, IMUPositionConfig, IMULiDARFusion,
    Quaternion, calculate_position_from_imu
)

try:
    from .complete_slam import CompleteSLAM, SLAMConfig
except ImportError as e:
    CompleteSLAM = None
    SLAMConfig = None
    print(f"Warning: Complete SLAM not available: {e}")

# Point-LIO: Tightly-coupled LiDAR-Inertial Odometry
try:
    from .point_lio import PointLIO, PointLIOConfig, PointLIOProcessor, create_point_lio, SO3
except ImportError as e:
    PointLIO = None
    PointLIOConfig = None
    PointLIOProcessor = None
    create_point_lio = None
    SO3 = None
    print(f"Warning: Point-LIO not available: {e}")

__all__ = [
    'PointCloudProcessor',
    'KISSICPOdometry',
    'IKDTree',
    'ScanContext',
    'ScanContextConfig',
    'PoseGraph',
    'CompleteSLAM',
    'SLAMConfig',
    'IMUIntegration',
    'IMUPreintegration',
    'IMUPositionEstimator',
    'IMUPositionConfig',
    'IMULiDARFusion',
    'Quaternion',
    'calculate_position_from_imu',
    'PointLIO',
    'PointLIOConfig',
    'PointLIOProcessor',
    'create_point_lio',
    'SO3'
]