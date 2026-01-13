"""
Point Cloud Processing
Core algorithms for processing and cleaning LiDAR point cloud data.
"""

import numpy as np
import open3d as o3d
import logging
from pathlib import Path
from typing import Optional, Tuple

logger = logging.getLogger(__name__)


class PointCloudProcessor:
    """
    Process and clean point cloud data from LiDAR scans.
    """

    def __init__(self):
        """Initialize the point cloud processor."""
        pass

    @staticmethod
    def remove_outliers(points: np.ndarray,
                       nb_neighbors: int = 20,
                       std_ratio: float = 2.0) -> np.ndarray:
        """
        Remove statistical outliers from point cloud.

        Args:
            points: Nx3 array of point coordinates
            nb_neighbors: Number of neighbors to analyze
            std_ratio: Standard deviation ratio threshold

        Returns:
            Filtered point cloud
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        pcd_filtered, _ = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio
        )

        return np.asarray(pcd_filtered.points)

    @staticmethod
    def downsample(points: np.ndarray, voxel_size: float = 0.1) -> np.ndarray:
        """
        Downsample point cloud using voxel grid.

        Args:
            points: Nx3 array of point coordinates
            voxel_size: Size of voxel grid cells

        Returns:
            Downsampled point cloud
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        pcd_downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)

        return np.asarray(pcd_downsampled.points)

    @staticmethod
    def estimate_normals(points: np.ndarray,
                        radius: float = 0.5,
                        max_nn: int = 30) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate surface normals for point cloud.

        Args:
            points: Nx3 array of point coordinates
            radius: Search radius for normal estimation
            max_nn: Maximum number of neighbors

        Returns:
            Tuple of (points, normals)
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radius,
                max_nn=max_nn
            )
        )

        normals = np.asarray(pcd.normals)
        return points, normals

    @staticmethod
    def segment_ground_plane(points: np.ndarray,
                            distance_threshold: float = 0.1,
                            ransac_n: int = 3,
                            num_iterations: int = 1000) -> Tuple[np.ndarray, np.ndarray]:
        """
        Segment ground plane from point cloud using RANSAC.

        Args:
            points: Nx3 array of point coordinates
            distance_threshold: Max distance for point to be considered inlier
            ransac_n: Number of points to sample for RANSAC
            num_iterations: Number of RANSAC iterations

        Returns:
            Tuple of (ground_points, non_ground_points)
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )

        ground_cloud = pcd.select_by_index(inliers)
        non_ground_cloud = pcd.select_by_index(inliers, invert=True)

        ground_points = np.asarray(ground_cloud.points)
        non_ground_points = np.asarray(non_ground_cloud.points)

        logger.info(f"Segmented ground plane: {len(ground_points)} ground, "
                   f"{len(non_ground_points)} non-ground points")

        return ground_points, non_ground_points

    @staticmethod
    def merge_point_clouds(point_clouds: list) -> np.ndarray:
        """
        Merge multiple point clouds into one.

        Args:
            point_clouds: List of Nx3 point cloud arrays

        Returns:
            Merged point cloud
        """
        if not point_clouds:
            return np.array([])

        merged = np.vstack(point_clouds)
        logger.info(f"Merged {len(point_clouds)} point clouds into {len(merged)} points")

        return merged

    def process_trail_scan(self,
                          raw_points: np.ndarray,
                          remove_outliers: bool = True,
                          downsample_voxel: Optional[float] = 0.05,
                          segment_ground: bool = True) -> dict:
        """
        Complete processing pipeline for trail scan data.

        Args:
            raw_points: Raw point cloud from sensor
            remove_outliers: Whether to remove statistical outliers
            downsample_voxel: Voxel size for downsampling (None to skip)
            segment_ground: Whether to segment ground plane

        Returns:
            Dictionary with processed point cloud data
        """
        logger.info(f"Processing trail scan with {len(raw_points)} raw points")

        result = {
            'raw_points': raw_points,
            'num_raw_points': len(raw_points)
        }

        points = raw_points.copy()

        # Remove outliers
        if remove_outliers:
            points = self.remove_outliers(points)
            logger.info(f"After outlier removal: {len(points)} points")
            result['after_outlier_removal'] = len(points)

        # Downsample
        if downsample_voxel is not None:
            points = self.downsample(points, voxel_size=downsample_voxel)
            logger.info(f"After downsampling: {len(points)} points")
            result['after_downsampling'] = len(points)

        # Segment ground
        if segment_ground:
            ground, non_ground = self.segment_ground_plane(points)
            result['ground_points'] = ground
            result['non_ground_points'] = non_ground
            result['num_ground_points'] = len(ground)
            result['num_non_ground_points'] = len(non_ground)
            points = non_ground  # Keep non-ground for trail features

        # Estimate normals for final point cloud
        points, normals = self.estimate_normals(points)

        result['processed_points'] = points
        result['normals'] = normals
        result['num_final_points'] = len(points)

        logger.info(f"Processing complete. Final: {len(points)} points")

        return result

    @staticmethod
    def save_processed_cloud(points: np.ndarray,
                            normals: Optional[np.ndarray],
                            output_path: str) -> None:
        """
        Save processed point cloud to file.

        Args:
            points: Nx3 point coordinates
            normals: Nx3 normal vectors (optional)
            output_path: Path to save the file
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        if normals is not None:
            pcd.normals = o3d.utility.Vector3dVector(normals)

        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        o3d.io.write_point_cloud(str(output_path), pcd)
        logger.info(f"Saved processed point cloud to {output_path}")