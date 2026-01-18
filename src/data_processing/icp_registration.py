"""
Iterative Closest Point (ICP) registration for LiDAR odometry
Estimates sensor movement by matching consecutive point clouds
"""
import numpy as np
from scipy.spatial import cKDTree
from typing import Tuple, Optional
import logging

logger = logging.getLogger(__name__)


class ICPRegistration:
    """
    Point cloud registration using Iterative Closest Point algorithm.
    Estimates the transformation between two point clouds.
    """

    def __init__(self, max_iterations: int = 50, tolerance: float = 1e-5,
                 max_correspondence_distance: float = 0.5):
        """
        Initialize ICP registration.

        Args:
            max_iterations: Maximum number of ICP iterations
            tolerance: Convergence tolerance
            max_correspondence_distance: Maximum distance to consider points as correspondences
        """
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.max_correspondence_distance = max_correspondence_distance

    def register(self, source: np.ndarray, target: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Register source point cloud to target using ICP.

        Args:
            source: Nx3 source point cloud
            target: Mx3 target point cloud

        Returns:
            Tuple of (4x4 transformation matrix, fitness score)
        """
        if len(source) < 10 or len(target) < 10:
            logger.warning("Not enough points for registration")
            return np.eye(4), 0.0

        # Downsample if too many points
        if len(source) > 5000:
            indices = np.random.choice(len(source), 5000, replace=False)
            source = source[indices]
        if len(target) > 5000:
            indices = np.random.choice(len(target), 5000, replace=False)
            target = target[indices]

        # Initialize transformation
        transformation = np.eye(4)
        prev_error = float('inf')

        # Build KD-tree for target
        tree = cKDTree(target)

        for iteration in range(self.max_iterations):
            # Find closest points
            distances, indices = tree.query(source, k=1)

            # Filter by distance threshold
            valid_mask = distances < self.max_correspondence_distance
            if valid_mask.sum() < 10:
                logger.warning(f"Too few correspondences: {valid_mask.sum()}")
                break

            source_matched = source[valid_mask]
            target_matched = target[indices[valid_mask]]

            # Compute transformation using SVD
            transform = self._compute_transformation(source_matched, target_matched)

            # Apply transformation
            source = self._apply_transformation(source, transform)
            transformation = transform @ transformation

            # Check convergence
            mean_error = distances[valid_mask].mean()
            error_change = abs(prev_error - mean_error)

            if error_change < self.tolerance:
                logger.debug(f"ICP converged at iteration {iteration+1}")
                break

            prev_error = mean_error

        # Calculate fitness (percentage of points with good matches)
        final_distances, _ = tree.query(source, k=1)
        fitness = (final_distances < self.max_correspondence_distance).sum() / len(source)

        return transformation, fitness

    def _compute_transformation(self, source: np.ndarray, target: np.ndarray) -> np.ndarray:
        """
        Compute rigid transformation between matched point sets using SVD.

        Args:
            source: Nx3 matched source points
            target: Nx3 matched target points

        Returns:
            4x4 transformation matrix
        """
        # Compute centroids
        source_centroid = source.mean(axis=0)
        target_centroid = target.mean(axis=0)

        # Center the points
        source_centered = source - source_centroid
        target_centered = target - target_centroid

        # Compute cross-covariance matrix
        H = source_centered.T @ target_centered

        # SVD
        U, S, Vt = np.linalg.svd(H)

        # Compute rotation
        R = Vt.T @ U.T

        # Handle reflection case
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # Compute translation
        t = target_centroid - R @ source_centroid

        # Build 4x4 transformation matrix
        transformation = np.eye(4)
        transformation[:3, :3] = R
        transformation[:3, 3] = t

        return transformation

    def _apply_transformation(self, points: np.ndarray, transformation: np.ndarray) -> np.ndarray:
        """
        Apply 4x4 transformation to points.

        Args:
            points: Nx3 point cloud
            transformation: 4x4 transformation matrix

        Returns:
            Transformed Nx3 point cloud
        """
        # Convert to homogeneous coordinates
        points_homogeneous = np.hstack([points, np.ones((len(points), 1))])

        # Apply transformation
        transformed = (transformation @ points_homogeneous.T).T

        # Convert back to 3D
        return transformed[:, :3]


class LiDAROdometry:
    """
    Estimates LiDAR sensor motion by registering consecutive scans.
    """

    def __init__(self, voxel_size: float = 0.05):
        """
        Initialize LiDAR odometry.

        Args:
            voxel_size: Voxel size for downsampling (meters)
        """
        self.voxel_size = voxel_size
        self.icp = ICPRegistration(
            max_iterations=30,
            tolerance=1e-6,
            max_correspondence_distance=0.3
        )
        self.pose = np.eye(4)  # Current pose in world frame
        self.previous_cloud = None

    def voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """
        Downsample point cloud using voxel grid.

        Args:
            points: Nx3 point cloud

        Returns:
            Downsampled point cloud
        """
        if len(points) < 100:
            return points

        # Compute voxel indices
        voxel_indices = np.floor(points / self.voxel_size).astype(int)

        # Find unique voxels
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)

        return points[unique_indices]

    def process_scan(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, bool]:
        """
        Process a new scan and estimate odometry.

        Args:
            points: Nx3 point cloud

        Returns:
            Tuple of (transformed points, current pose, success flag)
        """
        # Downsample
        points_down = self.voxel_downsample(points)

        if self.previous_cloud is None:
            # First scan - initialize
            self.previous_cloud = points_down
            return points_down, self.pose.copy(), True

        # Register current scan to previous
        transformation, fitness = self.icp.register(points_down, self.previous_cloud)

        # Check if registration was successful
        if fitness < 0.3:  # Less than 30% overlap
            logger.warning(f"Low registration fitness: {fitness:.2f}")
            # Keep current pose but update previous cloud
            self.previous_cloud = points_down
            return self._apply_pose(points), self.pose.copy(), False

        # Update pose (accumulate transformations)
        self.pose = self.pose @ transformation

        # Transform points to world frame
        transformed_points = self._apply_pose(points)

        # Update previous cloud
        self.previous_cloud = points_down

        logger.info(f"Odometry update - fitness: {fitness:.2f}, position: [{self.pose[0,3]:.2f}, {self.pose[1,3]:.2f}, {self.pose[2,3]:.2f}]")

        return transformed_points, self.pose.copy(), True

    def _apply_pose(self, points: np.ndarray) -> np.ndarray:
        """Apply current pose to points."""
        points_homogeneous = np.hstack([points, np.ones((len(points), 1))])
        transformed = (self.pose @ points_homogeneous.T).T
        return transformed[:, :3]

    def reset(self):
        """Reset odometry to initial state."""
        self.pose = np.eye(4)
        self.previous_cloud = None