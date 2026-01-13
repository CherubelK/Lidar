"""
Point Cloud Processing (NumPy-based implementation)
Works without Open3D - uses NumPy and scikit-learn instead.
"""

import numpy as np
from sklearn.neighbors import NearestNeighbors
from sklearn.linear_model import RANSACRegressor
import logging
from pathlib import Path
from typing import Optional, Tuple

logger = logging.getLogger(__name__)


class PointCloudProcessorNumPy:
    """
    Process and clean point cloud data using NumPy and scikit-learn.
    This is a fallback implementation for when Open3D is not available.
    """

    def __init__(self):
        """Initialize the point cloud processor."""
        pass

    @staticmethod
    def remove_outliers(points: np.ndarray,
                       nb_neighbors: int = 20,
                       std_ratio: float = 2.0) -> np.ndarray:
        """
        Remove statistical outliers from point cloud using k-NN.

        Args:
            points: Nx3 array of point coordinates
            nb_neighbors: Number of neighbors to analyze
            std_ratio: Standard deviation ratio threshold

        Returns:
            Filtered point cloud
        """
        if len(points) < nb_neighbors:
            logger.warning(f"Point cloud has fewer points ({len(points)}) than nb_neighbors ({nb_neighbors})")
            return points

        # Fit k-NN
        nbrs = NearestNeighbors(n_neighbors=nb_neighbors + 1).fit(points)
        distances, indices = nbrs.kneighbors(points)

        # Calculate mean distance to neighbors (excluding self)
        mean_distances = np.mean(distances[:, 1:], axis=1)

        # Compute threshold
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)
        threshold = global_mean + std_ratio * global_std

        # Filter
        mask = mean_distances < threshold
        filtered = points[mask]

        logger.info(f"Outlier removal: {len(points)} -> {len(filtered)} points ({len(points) - len(filtered)} removed)")
        return filtered

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
        if len(points) == 0:
            return points

        # Compute voxel indices for each point
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)

        # Get unique voxels
        unique_voxels, inverse_indices = np.unique(voxel_indices, axis=0, return_inverse=True)

        # Compute centroid for each voxel
        downsampled = np.zeros((len(unique_voxels), 3))
        for i in range(len(unique_voxels)):
            mask = inverse_indices == i
            downsampled[i] = np.mean(points[mask], axis=0)

        logger.info(f"Downsampling: {len(points)} -> {len(downsampled)} points")
        return downsampled

    @staticmethod
    def estimate_normals(points: np.ndarray,
                        k_neighbors: int = 30) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate surface normals using PCA on local neighborhoods.

        Args:
            points: Nx3 array of point coordinates
            k_neighbors: Number of neighbors for normal estimation

        Returns:
            Tuple of (points, normals)
        """
        if len(points) < k_neighbors:
            logger.warning(f"Point cloud has fewer points than k_neighbors")
            k_neighbors = max(3, len(points) // 2)

        # Fit k-NN
        nbrs = NearestNeighbors(n_neighbors=k_neighbors).fit(points)
        _, indices = nbrs.kneighbors(points)

        normals = np.zeros_like(points)

        for i in range(len(points)):
            # Get neighborhood
            neighbors = points[indices[i]]

            # Center the neighborhood
            centered = neighbors - np.mean(neighbors, axis=0)

            # PCA - eigenvector with smallest eigenvalue is normal
            cov = np.cov(centered.T)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)

            # Normal is eigenvector with smallest eigenvalue
            normal = eigenvectors[:, 0]
            normals[i] = normal

        # Normalize
        norms = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = normals / (norms + 1e-10)

        logger.info(f"Estimated normals for {len(points)} points")
        return points, normals

    @staticmethod
    def segment_ground_plane(points: np.ndarray,
                            distance_threshold: float = 0.1,
                            max_iterations: int = 1000,
                            min_samples: int = 3) -> Tuple[np.ndarray, np.ndarray]:
        """
        Segment ground plane from point cloud using RANSAC.

        Args:
            points: Nx3 array of point coordinates
            distance_threshold: Max distance for point to be considered inlier
            max_iterations: Number of RANSAC iterations
            min_samples: Minimum number of samples

        Returns:
            Tuple of (ground_points, non_ground_points)
        """
        if len(points) < min_samples:
            logger.warning("Not enough points for ground segmentation")
            return np.array([]).reshape(0, 3), points

        # Use XY as features, Z as target (assuming Z is vertical)
        X = points[:, :2]  # X, Y coordinates
        y = points[:, 2]   # Z coordinate

        # RANSAC to fit plane
        ransac = RANSACRegressor(
            max_trials=max_iterations,
            residual_threshold=distance_threshold,
            min_samples=min_samples,
            random_state=42
        )

        try:
            ransac.fit(X, y)
            inlier_mask = ransac.inlier_mask_

            ground_points = points[inlier_mask]
            non_ground_points = points[~inlier_mask]

            logger.info(f"Ground segmentation: {len(ground_points)} ground, {len(non_ground_points)} non-ground")
            return ground_points, non_ground_points

        except Exception as e:
            logger.error(f"Ground segmentation failed: {e}")
            return np.array([]).reshape(0, 3), points

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
            return np.array([]).reshape(0, 3)

        # Filter out empty clouds
        valid_clouds = [pc for pc in point_clouds if len(pc) > 0]

        if not valid_clouds:
            return np.array([]).reshape(0, 3)

        merged = np.vstack(valid_clouds)
        logger.info(f"Merged {len(valid_clouds)} point clouds into {len(merged)} points")

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
            result['after_outlier_removal'] = len(points)

        # Downsample
        if downsample_voxel is not None and len(points) > 0:
            points = self.downsample(points, voxel_size=downsample_voxel)
            result['after_downsampling'] = len(points)

        # Segment ground
        if segment_ground and len(points) > 0:
            ground, non_ground = self.segment_ground_plane(points)
            result['ground_points'] = ground
            result['non_ground_points'] = non_ground
            result['num_ground_points'] = len(ground)
            result['num_non_ground_points'] = len(non_ground)
            points = non_ground if len(non_ground) > 0 else ground

        # Estimate normals for final point cloud
        if len(points) > 0:
            points, normals = self.estimate_normals(points)
            result['normals'] = normals
        else:
            result['normals'] = np.array([]).reshape(0, 3)

        result['processed_points'] = points
        result['num_final_points'] = len(points)

        logger.info(f"Processing complete. Final: {len(points)} points")

        return result

    @staticmethod
    def save_processed_cloud(points: np.ndarray,
                            normals: Optional[np.ndarray],
                            output_path: str) -> None:
        """
        Save processed point cloud to file (NumPy format).

        Args:
            points: Nx3 point coordinates
            normals: Nx3 normal vectors (optional)
            output_path: Path to save the file
        """
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Save as NPZ with points and normals
        if normals is not None:
            np.savez(str(output_path).replace('.pcd', '.npz'),
                    points=points,
                    normals=normals)
        else:
            np.savez(str(output_path).replace('.pcd', '.npz'),
                    points=points)

        logger.info(f"Saved processed point cloud to {output_path}")

    @staticmethod
    def load_processed_cloud(input_path: str) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        Load processed point cloud from file.

        Args:
            input_path: Path to the NPZ file

        Returns:
            Tuple of (points, normals)
        """
        data = np.load(input_path)
        points = data['points']
        normals = data['normals'] if 'normals' in data else None

        logger.info(f"Loaded {len(points)} points from {input_path}")
        return points, normals