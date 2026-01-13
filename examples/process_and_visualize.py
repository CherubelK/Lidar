"""
Example: Process and Visualize Point Cloud Data

This example shows how to load, process, and visualize captured LiDAR data.
"""

import sys
from pathlib import Path
import numpy as np

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.data_processing import PointCloudProcessor
from src.visualization.visualizer import PointCloudVisualizer
import logging

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


def main():
    """Load, process, and visualize point cloud data."""

    # Example: Generate synthetic data for testing
    # Replace this with actual data loading once you have captures
    print("Generating synthetic test data...")
    raw_points = generate_test_trail_data()

    # Initialize processor
    processor = PointCloudProcessor()

    # Process the data
    print("Processing point cloud...")
    result = processor.process_trail_scan(
        raw_points,
        remove_outliers=True,
        downsample_voxel=0.05,
        segment_ground=True
    )

    print(f"\nProcessing Results:")
    print(f"  Raw points: {result['num_raw_points']}")
    print(f"  After filtering: {result['num_final_points']}")
    print(f"  Ground points: {result['num_ground_points']}")
    print(f"  Non-ground points: {result['num_non_ground_points']}")

    # Save processed data
    output_path = "data/processed/test_trail_processed.pcd"
    processor.save_processed_cloud(
        result['processed_points'],
        result['normals'],
        output_path
    )
    print(f"\nSaved processed data to: {output_path}")

    # Visualize
    print("\nVisualizing point cloud...")
    visualizer = PointCloudVisualizer()

    # Show ground vs non-ground
    visualizer.visualize_multiple_clouds(
        [result['ground_points'], result['non_ground_points']],
        labels=['Ground', 'Trail Features'],
        colors=[[0.5, 0.5, 0.5], [0.2, 0.8, 0.2]]
    )

    # Plot statistics
    visualizer.plot_point_cloud_stats(
        result['processed_points'],
        save_path="trail_stats.png"
    )


def generate_test_trail_data(num_points=10000):
    """
    Generate synthetic trail data for testing.

    This simulates a simple hiking trail with ground plane and some obstacles.
    """
    np.random.seed(42)

    # Create ground plane (trail surface)
    x = np.random.uniform(-5, 5, num_points // 2)
    y = np.random.uniform(0, 20, num_points // 2)
    z = np.sin(y * 0.3) * 0.5 + np.random.normal(0, 0.05, num_points // 2)  # Slight elevation changes

    ground_points = np.column_stack([x, y, z])

    # Add some trail features (rocks, trees, etc.)
    num_features = num_points // 2
    feature_x = np.random.uniform(-4, 4, num_features)
    feature_y = np.random.uniform(0, 20, num_features)
    feature_z = np.random.uniform(0.2, 3.0, num_features)  # Above ground

    feature_points = np.column_stack([feature_x, feature_y, feature_z])

    # Combine
    all_points = np.vstack([ground_points, feature_points])

    # Add some noise
    all_points += np.random.normal(0, 0.02, all_points.shape)

    return all_points


if __name__ == "__main__":
    main()