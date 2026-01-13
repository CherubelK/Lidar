"""
Data Processing Module
Point cloud processing, filtering, and transformation utilities.
"""

try:
    from .point_cloud_processor import PointCloudProcessor
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    # Fall back to NumPy-based processor
    from .point_cloud_processor_numpy import PointCloudProcessorNumPy as PointCloudProcessor

__all__ = ['PointCloudProcessor']