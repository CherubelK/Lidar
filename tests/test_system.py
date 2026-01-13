"""
System integration tests for the trail mapping pipeline
"""

import pytest
import numpy as np
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.data_processing.point_cloud_processor_numpy import PointCloudProcessorNumPy
from src.data_processing.mesh_generator import MeshGenerator


def generate_test_data(num_points=1000):
    """Generate synthetic test data."""
    np.random.seed(42)
    x = np.random.uniform(-5, 5, num_points)
    y = np.random.uniform(0, 10, num_points)
    z = np.sin(y * 0.3) * 0.5 + np.random.normal(0, 0.05, num_points)
    return np.column_stack([x, y, z])


def test_point_cloud_processor():
    """Test point cloud processing pipeline."""
    processor = PointCloudProcessorNumPy()
    test_data = generate_test_data()

    # Test outlier removal
    filtered = processor.remove_outliers(test_data, nb_neighbors=10)
    assert len(filtered) <= len(test_data)
    assert len(filtered) > 0

    # Test downsampling
    downsampled = processor.downsample(test_data, voxel_size=0.5)
    assert len(downsampled) <= len(test_data)
    assert len(downsampled) > 0

    # Test normal estimation
    points, normals = processor.estimate_normals(test_data[:100])
    assert points.shape == (100, 3)
    assert normals.shape == (100, 3)

    # Test ground segmentation
    ground, non_ground = processor.segment_ground_plane(test_data)
    assert len(ground) + len(non_ground) == len(test_data)


def test_complete_pipeline():
    """Test complete processing pipeline."""
    processor = PointCloudProcessorNumPy()
    test_data = generate_test_data(500)

    result = processor.process_trail_scan(
        test_data,
        remove_outliers=True,
        downsample_voxel=0.1,
        segment_ground=True
    )

    assert 'processed_points' in result
    assert 'normals' in result
    assert 'num_final_points' in result
    assert result['num_raw_points'] == len(test_data)
    assert len(result['processed_points']) > 0


def test_mesh_generation():
    """Test mesh generation from point cloud."""
    processor = PointCloudProcessorNumPy()
    generator = MeshGenerator()

    # Generate and process test data
    test_data = generate_test_data(200)
    result = processor.process_trail_scan(
        test_data,
        remove_outliers=False,
        downsample_voxel=0.2,
        segment_ground=False
    )

    # Create mesh
    mesh_data = generator.create_trail_mesh(
        result['processed_points'],
        result['normals'],
        method='delaunay'
    )

    assert 'vertices' in mesh_data
    assert 'faces' in mesh_data
    assert 'num_vertices' in mesh_data
    assert 'num_faces' in mesh_data
    assert mesh_data['num_vertices'] > 0
    assert mesh_data['num_faces'] >= 0


def test_merge_point_clouds():
    """Test merging multiple point clouds."""
    processor = PointCloudProcessorNumPy()

    cloud1 = generate_test_data(100)
    cloud2 = generate_test_data(100)
    cloud3 = generate_test_data(100)

    merged = processor.merge_point_clouds([cloud1, cloud2, cloud3])

    assert len(merged) == 300
    assert merged.shape[1] == 3


if __name__ == "__main__":
    pytest.main([__file__, "-v"])