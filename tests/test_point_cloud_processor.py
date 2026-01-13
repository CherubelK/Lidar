"""
Unit tests for Point Cloud Processor
"""

import pytest
import numpy as np
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.data_processing.point_cloud_processor import PointCloudProcessor


@pytest.fixture
def sample_point_cloud():
    """Generate a sample point cloud for testing."""
    np.random.seed(42)
    points = np.random.randn(1000, 3) * 5.0
    return points


@pytest.fixture
def processor():
    """Create a PointCloudProcessor instance."""
    return PointCloudProcessor()


def test_remove_outliers(processor, sample_point_cloud):
    """Test outlier removal."""
    filtered = processor.remove_outliers(sample_point_cloud)

    assert filtered.shape[1] == 3, "Should have 3 dimensions"
    assert len(filtered) <= len(sample_point_cloud), "Filtered cloud should be smaller or equal"
    assert len(filtered) > 0, "Should have some points remaining"


def test_downsample(processor, sample_point_cloud):
    """Test downsampling."""
    downsampled = processor.downsample(sample_point_cloud, voxel_size=0.5)

    assert downsampled.shape[1] == 3, "Should have 3 dimensions"
    assert len(downsampled) <= len(sample_point_cloud), "Downsampled cloud should be smaller"
    assert len(downsampled) > 0, "Should have some points remaining"


def test_estimate_normals(processor, sample_point_cloud):
    """Test normal estimation."""
    points, normals = processor.estimate_normals(sample_point_cloud)

    assert points.shape == sample_point_cloud.shape, "Points should be unchanged"
    assert normals.shape == sample_point_cloud.shape, "Normals should match point count"

    # Check that normals are unit vectors
    norms = np.linalg.norm(normals, axis=1)
    assert np.allclose(norms, 1.0, atol=0.1), "Normals should be approximately unit vectors"


def test_segment_ground_plane(processor):
    """Test ground plane segmentation."""
    # Create synthetic data with clear ground plane
    np.random.seed(42)

    # Ground points (z ≈ 0)
    ground = np.random.randn(500, 3)
    ground[:, 2] = np.random.normal(0, 0.05, 500)

    # Non-ground points (z > 1)
    non_ground = np.random.randn(500, 3)
    non_ground[:, 2] = np.random.uniform(1, 5, 500)

    all_points = np.vstack([ground, non_ground])

    ground_result, non_ground_result = processor.segment_ground_plane(all_points)

    assert len(ground_result) > 0, "Should find ground points"
    assert len(non_ground_result) > 0, "Should find non-ground points"
    assert len(ground_result) + len(non_ground_result) == len(all_points), "Should account for all points"


def test_merge_point_clouds(processor):
    """Test merging multiple point clouds."""
    cloud1 = np.random.randn(100, 3)
    cloud2 = np.random.randn(150, 3)
    cloud3 = np.random.randn(200, 3)

    merged = processor.merge_point_clouds([cloud1, cloud2, cloud3])

    assert len(merged) == 450, "Merged cloud should have sum of all points"
    assert merged.shape[1] == 3, "Should maintain 3D structure"


def test_process_trail_scan(processor, sample_point_cloud):
    """Test the complete processing pipeline."""
    result = processor.process_trail_scan(
        sample_point_cloud,
        remove_outliers=True,
        downsample_voxel=0.1,
        segment_ground=True
    )

    assert 'processed_points' in result
    assert 'normals' in result
    assert 'num_final_points' in result
    assert result['num_raw_points'] == len(sample_point_cloud)
    assert len(result['processed_points']) > 0
    assert result['processed_points'].shape[1] == 3


def test_empty_point_cloud(processor):
    """Test handling of empty point clouds."""
    empty = np.array([]).reshape(0, 3)
    merged = processor.merge_point_clouds([empty])

    assert len(merged) == 0, "Should handle empty clouds gracefully"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])