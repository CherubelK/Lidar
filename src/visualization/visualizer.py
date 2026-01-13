"""
3D Visualization tools for point cloud data
"""

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from pathlib import Path
import logging

logger = logging.getLogger(__name__)


class PointCloudVisualizer:
    """
    Visualize point cloud data in 3D.
    """

    @staticmethod
    def visualize_point_cloud(points: np.ndarray,
                             normals: np.ndarray = None,
                             colors: np.ndarray = None,
                             window_name: str = "Point Cloud") -> None:
        """
        Display point cloud in interactive 3D viewer.

        Args:
            points: Nx3 array of point coordinates
            normals: Nx3 array of normal vectors (optional)
            colors: Nx3 array of RGB colors (optional)
            window_name: Window title
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        if normals is not None:
            pcd.normals = o3d.utility.Vector3dVector(normals)

        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors)
        else:
            # Default: color by height (z-coordinate)
            z_values = points[:, 2]
            z_normalized = (z_values - z_values.min()) / (z_values.max() - z_values.min())
            colors = plt.cm.viridis(z_normalized)[:, :3]
            pcd.colors = o3d.utility.Vector3dVector(colors)

        o3d.visualization.draw_geometries(
            [pcd],
            window_name=window_name,
            width=1024,
            height=768
        )

    @staticmethod
    def visualize_multiple_clouds(point_clouds: list,
                                  labels: list = None,
                                  colors: list = None) -> None:
        """
        Visualize multiple point clouds together.

        Args:
            point_clouds: List of Nx3 point arrays
            labels: List of labels for each cloud
            colors: List of RGB colors for each cloud
        """
        geometries = []

        if colors is None:
            # Generate distinct colors
            colormap = plt.cm.get_cmap('tab10')
            colors = [colormap(i)[:3] for i in range(len(point_clouds))]

        for i, points in enumerate(point_clouds):
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            color = np.array(colors[i])
            pcd.paint_uniform_color(color)

            geometries.append(pcd)

        o3d.visualization.draw_geometries(
            geometries,
            window_name="Multiple Point Clouds",
            width=1024,
            height=768
        )

    @staticmethod
    def plot_point_cloud_stats(points: np.ndarray, save_path: str = None) -> None:
        """
        Plot statistical information about the point cloud.

        Args:
            points: Nx3 array of point coordinates
            save_path: Optional path to save the figure
        """
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        # XYZ histograms
        for i, (ax, label) in enumerate(zip(axes.flat[:3], ['X', 'Y', 'Z'])):
            ax.hist(points[:, i], bins=50, edgecolor='black', alpha=0.7)
            ax.set_xlabel(f'{label} coordinate (m)')
            ax.set_ylabel('Frequency')
            ax.set_title(f'{label} Distribution')
            ax.grid(True, alpha=0.3)

        # Point density
        ax = axes[1, 1]
        ax.scatter(points[:, 0], points[:, 1], c=points[:, 2],
                  cmap='viridis', s=1, alpha=0.5)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Top-down view (colored by height)')
        ax.set_aspect('equal')
        plt.colorbar(ax.collections[0], ax=ax, label='Z (m)')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            logger.info(f"Saved statistics plot to {save_path}")

        plt.show()

    @staticmethod
    def create_trail_video(point_clouds: list,
                          output_path: str,
                          fps: int = 10) -> None:
        """
        Create video animation from sequence of point clouds.

        Args:
            point_clouds: List of point cloud arrays
            output_path: Path to save video file
            fps: Frames per second
        """
        # TODO: Implement video creation
        # This would use Open3D's VisualizerWithKeyCallback
        # or matplotlib animation to create video

        logger.warning("Video creation not yet implemented")
        pass