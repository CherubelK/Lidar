"""
Map merging and synchronization for multi-device LiDAR scanning
Combines point clouds from multiple devices into unified map
"""

import numpy as np
import open3d as o3d
import lz4.frame
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from datetime import datetime


@dataclass
class DeviceMap:
    """Point cloud map from a single device"""
    device_id: str
    points: np.ndarray  # Nx3 array
    intensities: Optional[np.ndarray] = None  # N array
    gps_origin: Optional[Tuple[float, float, float]] = None  # (lat, lon, alt)
    timestamp: datetime = None
    has_gps: bool = False

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()


class MapMerger:
    """
    Merges point clouds from multiple LiDAR devices

    Supports three merging strategies:
    1. GPS-based (outdoor): Use GPS coordinates for global alignment
    2. ICP-based (indoor): Use point cloud registration
    3. Hybrid (best): Coarse GPS alignment + fine ICP refinement
    """

    def __init__(self, strategy: str = 'hybrid', voxel_size: float = 0.05):
        """
        Initialize map merger

        Args:
            strategy: Merge strategy ('gps', 'icp', or 'hybrid')
            voxel_size: Voxel grid size for downsampling (meters)
        """
        if strategy not in ['gps', 'icp', 'hybrid']:
            raise ValueError(f"Invalid strategy: {strategy}")

        self.strategy = strategy
        self.voxel_size = voxel_size

        # Merged map
        self.merged_points: Optional[np.ndarray] = None
        self.merged_intensities: Optional[np.ndarray] = None

        # GPS reference point (for outdoor mapping)
        self.gps_reference: Optional[Tuple[float, float, float]] = None

    def add_device_map(self, device_map: DeviceMap) -> bool:
        """
        Add a device's map to the merged map

        Args:
            device_map: Map from a single device

        Returns:
            True if successfully merged
        """
        print(f"\nMerging map from {device_map.device_id}...")
        print(f"  Points: {len(device_map.points):,}")
        print(f"  Has GPS: {device_map.has_gps}")

        # First map - use as reference
        if self.merged_points is None:
            print("  First map - using as reference")
            self.merged_points = device_map.points.copy()
            self.merged_intensities = device_map.intensities

            if device_map.has_gps:
                self.gps_reference = device_map.gps_origin

            return True

        # Merge based on strategy
        if self.strategy == 'gps' and device_map.has_gps:
            return self._merge_gps(device_map)
        elif self.strategy == 'icp':
            return self._merge_icp(device_map)
        elif self.strategy == 'hybrid':
            return self._merge_hybrid(device_map)
        else:
            print(f"  WARNING: Cannot merge with strategy '{self.strategy}' (no GPS)")
            return self._merge_icp(device_map)  # Fallback to ICP

    def _convert_gps_to_local(self, points: np.ndarray,
                             gps_origin: Tuple[float, float, float]) -> np.ndarray:
        """
        Convert GPS coordinates to local metric coordinates

        Args:
            points: Nx3 array in local frame
            gps_origin: (latitude, longitude, altitude) of device origin

        Returns:
            Points in global UTM-like coordinate system
        """
        if self.gps_reference is None:
            raise ValueError("No GPS reference set")

        # Calculate offset from reference point
        ref_lat, ref_lon, ref_alt = self.gps_reference
        dev_lat, dev_lon, dev_alt = gps_origin

        # Convert to meters (approximate)
        meters_per_degree_lat = 111320.0  # meters per degree latitude
        meters_per_degree_lon = 111320.0 * np.cos(np.radians(ref_lat))

        # Calculate device origin offset in meters
        offset_x = (dev_lon - ref_lon) * meters_per_degree_lon
        offset_y = (dev_lat - ref_lat) * meters_per_degree_lat
        offset_z = dev_alt - ref_alt

        # Transform points
        global_points = points.copy()
        global_points[:, 0] += offset_x
        global_points[:, 1] += offset_y
        global_points[:, 2] += offset_z

        return global_points

    def _merge_gps(self, device_map: DeviceMap) -> bool:
        """Merge using GPS coordinates"""
        print("  Merging with GPS alignment...")

        if not device_map.has_gps or self.gps_reference is None:
            print("  ERROR: GPS data required for GPS merge strategy")
            return False

        # Convert device points to global coordinates
        global_points = self._convert_gps_to_local(
            device_map.points,
            device_map.gps_origin
        )

        # Concatenate with merged map
        self.merged_points = np.vstack([self.merged_points, global_points])

        if device_map.intensities is not None and self.merged_intensities is not None:
            self.merged_intensities = np.hstack([
                self.merged_intensities,
                device_map.intensities
            ])

        print(f"  ✓ Merged with GPS (now {len(self.merged_points):,} points)")
        return True

    def _merge_icp(self, device_map: DeviceMap) -> bool:
        """Merge using ICP registration"""
        print("  Merging with ICP registration...")

        # Create point clouds
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(device_map.points)

        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(self.merged_points)

        # Downsample for faster registration
        source_down = source.voxel_down_sample(self.voxel_size)
        target_down = target.voxel_down_sample(self.voxel_size)

        # Estimate normals
        source_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 2, max_nn=30
            )
        )
        target_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 2, max_nn=30
            )
        )

        # Initial alignment with RANSAC
        print("    Running RANSAC for initial alignment...")
        ransac_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down,
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500),
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            ransac_n=4,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(self.voxel_size * 1.5)
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
        )

        # Refine with ICP
        print("    Running ICP for fine alignment...")
        threshold = self.voxel_size * 2
        reg_icp = o3d.pipelines.registration.registration_icp(
            source_down, target_down,
            threshold,
            ransac_result.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
        )

        print(f"    ICP fitness: {reg_icp.fitness:.3f}, RMSE: {reg_icp.inlier_rmse:.3f}m")

        # Check if registration was successful
        if reg_icp.fitness < 0.3:
            print(f"  WARNING: Low ICP fitness ({reg_icp.fitness:.3f}) - registration may be poor")

        # Transform original source points
        source.transform(reg_icp.transformation)

        # Merge
        transformed_points = np.asarray(source.points)
        self.merged_points = np.vstack([self.merged_points, transformed_points])

        if device_map.intensities is not None and self.merged_intensities is not None:
            self.merged_intensities = np.hstack([
                self.merged_intensities,
                device_map.intensities
            ])

        print(f"  ✓ Merged with ICP (now {len(self.merged_points):,} points)")
        return True

    def _merge_hybrid(self, device_map: DeviceMap) -> bool:
        """Merge using GPS for coarse alignment + ICP for refinement"""
        print("  Merging with Hybrid (GPS + ICP)...")

        if device_map.has_gps and self.gps_reference is not None:
            # Use GPS for initial alignment
            print("    Phase 1: GPS coarse alignment...")
            global_points = self._convert_gps_to_local(
                device_map.points,
                device_map.gps_origin
            )

            # Create temporary map for ICP refinement
            temp_map = DeviceMap(
                device_id=device_map.device_id,
                points=global_points,
                intensities=device_map.intensities,
                has_gps=False
            )

            # Refine with ICP
            print("    Phase 2: ICP fine alignment...")
            return self._merge_icp(temp_map)

        else:
            # No GPS - fall back to ICP only
            print("    No GPS available, using ICP only...")
            return self._merge_icp(device_map)

    def get_merged_map(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        Get the merged point cloud

        Returns:
            (points, intensities) tuple
        """
        if self.merged_points is None:
            return np.array([]), None

        return self.merged_points, self.merged_intensities

    def downsample_merged(self, voxel_size: Optional[float] = None) -> None:
        """
        Downsample merged map to reduce size

        Args:
            voxel_size: Voxel size (defaults to self.voxel_size)
        """
        if self.merged_points is None:
            return

        if voxel_size is None:
            voxel_size = self.voxel_size

        print(f"\nDownsampling merged map (voxel size: {voxel_size}m)...")
        print(f"  Before: {len(self.merged_points):,} points")

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.merged_points)

        # Add intensities as colors if available
        if self.merged_intensities is not None:
            colors = np.zeros((len(self.merged_intensities), 3))
            colors[:, 0] = self.merged_intensities / 255.0  # Red channel
            pcd.colors = o3d.utility.Vector3dVector(colors)

        # Downsample
        downsampled = pcd.voxel_down_sample(voxel_size)

        self.merged_points = np.asarray(downsampled.points)

        if self.merged_intensities is not None:
            colors = np.asarray(downsampled.colors)
            self.merged_intensities = (colors[:, 0] * 255).astype(np.uint8)

        print(f"  After: {len(self.merged_points):,} points")
        print(f"  Reduction: {(1 - len(self.merged_points) / len(pcd.points)) * 100:.1f}%")

    def remove_outliers(self, nb_neighbors: int = 20, std_ratio: float = 2.0) -> None:
        """
        Remove statistical outliers from merged map

        Args:
            nb_neighbors: Number of neighbors to analyze
            std_ratio: Standard deviation ratio threshold
        """
        if self.merged_points is None:
            return

        print(f"\nRemoving outliers...")
        print(f"  Before: {len(self.merged_points):,} points")

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.merged_points)

        # Statistical outlier removal
        pcd_clean, inlier_indices = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio
        )

        self.merged_points = np.asarray(pcd_clean.points)

        if self.merged_intensities is not None:
            self.merged_intensities = self.merged_intensities[inlier_indices]

        print(f"  After: {len(self.merged_points):,} points")
        print(f"  Removed: {len(pcd.points) - len(self.merged_points):,} outliers")

    def save_merged_map(self, output_dir: str, prefix: str = "merged") -> None:
        """
        Save merged map to disk

        Args:
            output_dir: Output directory
            prefix: Filename prefix
        """
        from pathlib import Path
        import json

        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # Save points
        points_file = output_path / f"{prefix}_points.npy"
        np.save(points_file, self.merged_points)
        print(f"Saved points: {points_file}")

        # Save intensities
        if self.merged_intensities is not None:
            intensity_file = output_path / f"{prefix}_intensity.npy"
            np.save(intensity_file, self.merged_intensities)
            print(f"Saved intensities: {intensity_file}")

        # Save PLY
        ply_file = output_path / f"{prefix}.ply"
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.merged_points)

        if self.merged_intensities is not None:
            colors = np.zeros((len(self.merged_intensities), 3))
            colors[:, 0] = self.merged_intensities / 255.0
            pcd.colors = o3d.utility.Vector3dVector(colors)

        o3d.io.write_point_cloud(str(ply_file), pcd)
        print(f"Saved PLY: {ply_file}")

        # Save metadata
        metadata = {
            'strategy': self.strategy,
            'voxel_size': self.voxel_size,
            'total_points': len(self.merged_points),
            'has_intensities': self.merged_intensities is not None,
            'gps_reference': self.gps_reference,
            'timestamp': datetime.now().isoformat()
        }

        metadata_file = output_path / f"{prefix}_metadata.json"
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)
        print(f"Saved metadata: {metadata_file}")

    def get_stats(self) -> dict:
        """Get statistics about merged map"""
        if self.merged_points is None:
            return {'points': 0, 'merged': False}

        return {
            'points': len(self.merged_points),
            'merged': True,
            'strategy': self.strategy,
            'voxel_size': self.voxel_size,
            'has_intensities': self.merged_intensities is not None,
            'has_gps_reference': self.gps_reference is not None,
            'bounds': {
                'x_min': float(self.merged_points[:, 0].min()),
                'x_max': float(self.merged_points[:, 0].max()),
                'y_min': float(self.merged_points[:, 1].min()),
                'y_max': float(self.merged_points[:, 1].max()),
                'z_min': float(self.merged_points[:, 2].min()),
                'z_max': float(self.merged_points[:, 2].max()),
            }
        }


def compress_pointcloud(points: np.ndarray, voxel_size: float = 0.05) -> bytes:
    """
    Compress point cloud for network transmission

    Args:
        points: Nx3 array
        voxel_size: Voxel downsample size

    Returns:
        Compressed bytes (LZ4 + quantization)
    """
    # Voxel downsample
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    downsampled = pcd.voxel_down_sample(voxel_size)
    points_ds = np.asarray(downsampled.points)

    # Quantize to uint16 (2 bytes per coordinate)
    min_coords = points_ds.min(axis=0)
    max_coords = points_ds.max(axis=0)
    scale = (max_coords - min_coords) / 65535

    quantized = ((points_ds - min_coords) / scale).astype(np.uint16)

    # Store metadata + quantized data
    metadata = np.array([
        min_coords[0], min_coords[1], min_coords[2],
        scale[0], scale[1], scale[2]
    ], dtype=np.float32)

    data = np.concatenate([metadata.tobytes(), quantized.tobytes()])

    # LZ4 compress
    compressed = lz4.frame.compress(data)

    return compressed


def decompress_pointcloud(compressed: bytes) -> np.ndarray:
    """
    Decompress point cloud

    Args:
        compressed: Compressed bytes from compress_pointcloud()

    Returns:
        Nx3 array of points
    """
    # Decompress
    data = lz4.frame.decompress(compressed)

    # Extract metadata
    metadata = np.frombuffer(data[:24], dtype=np.float32)
    min_coords = metadata[:3]
    scale = metadata[3:]

    # Dequantize
    quantized = np.frombuffer(data[24:], dtype=np.uint16).reshape(-1, 3)
    points = quantized.astype(np.float32) * scale + min_coords

    return points


if __name__ == "__main__":
    print("Map merger module - use examples/multi_device_scan.py for demo")