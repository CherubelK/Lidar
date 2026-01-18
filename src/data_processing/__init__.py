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
"""

try:
    from .point_cloud_processor import PointCloudProcessor
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    # Fall back to NumPy-based processor
    from .point_cloud_processor_numpy import PointCloudProcessorNumPy as PointCloudProcessor

# SLAM components
from .kiss_icp_odometry import KISSICPOdometry
from .ikd_tree import IKDTree
from .scan_context import ScanContext, ScanContextConfig
from .pose_graph import PoseGraph
from .complete_slam import CompleteSLAM, SLAMConfig

__all__ = [
    'PointCloudProcessor',
    'KISSICPOdometry',
    'IKDTree',
    'ScanContext',
    'ScanContextConfig',
    'PoseGraph',
    'CompleteSLAM',
    'SLAMConfig'
]