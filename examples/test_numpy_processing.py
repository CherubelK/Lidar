"""
Example: Test Point Cloud Processing with NumPy-based implementation

This example works without Open3D, using NumPy and scikit-learn instead.
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.data_processing.point_cloud_processor_numpy import PointCloudProcessorNumPy
import logging

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


def generate_test_trail_data(num_points=10000):
    """
    Generate synthetic trail data for testing.

    This simulates a simple hiking trail with ground plane and some obstacles.
    """
    print("Generating synthetic test data...")
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


def plot_point_cloud_2d(points, title="Point Cloud"):
    """Simple 2D visualization of point cloud."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Top-down view
    scatter1 = ax1.scatter(points[:, 0], points[:, 1], c=points[:, 2],
                          cmap='viridis', s=1, alpha=0.5)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title(f'{title} - Top View')
    ax1.set_aspect('equal')
    plt.colorbar(scatter1, ax=ax1, label='Z (m)')

    # Side view
    scatter2 = ax2.scatter(points[:, 1], points[:, 2], c=points[:, 0],
                          cmap='plasma', s=1, alpha=0.5)
    ax2.set_xlabel('Y (m)')
    ax2.set_ylabel('Z (m)')
    ax2.set_title(f'{title} - Side View')
    plt.colorbar(scatter2, ax=ax2, label='X (m)')

    plt.tight_layout()
    return fig


def main():
    """Run the processing pipeline test."""

    # Generate synthetic data
    raw_points = generate_test_trail_data(num_points=10000)
    print(f"Generated {len(raw_points)} test points")

    # Initialize processor
    processor = PointCloudProcessorNumPy()

    # Process the data
    print("\nProcessing point cloud...")
    result = processor.process_trail_scan(
        raw_points,
        remove_outliers=True,
        downsample_voxel=0.05,
        segment_ground=True
    )

    # Display results
    print(f"\n{'='*50}")
    print("Processing Results:")
    print(f"{'='*50}")
    print(f"  Raw points:          {result['num_raw_points']}")
    if 'after_outlier_removal' in result:
        print(f"  After outliers:      {result['after_outlier_removal']}")
    if 'after_downsampling' in result:
        print(f"  After downsampling:  {result['after_downsampling']}")
    print(f"  Ground points:       {result['num_ground_points']}")
    print(f"  Non-ground points:   {result['num_non_ground_points']}")
    print(f"  Final points:        {result['num_final_points']}")
    print(f"{'='*50}\n")

    # Save processed data
    output_path = "data/processed/test_trail_numpy.npz"
    processor.save_processed_cloud(
        result['processed_points'],
        result['normals'],
        output_path
    )
    print(f"Saved processed data to: {output_path}")

    # Visualize results
    print("\nGenerating visualizations...")

    # Plot raw data
    fig1 = plot_point_cloud_2d(raw_points, "Raw Point Cloud")
    plt.savefig("output_raw_points.png", dpi=150, bbox_inches='tight')
    print("Saved: output_raw_points.png")

    # Plot ground vs non-ground
    if result['num_ground_points'] > 0 and result['num_non_ground_points'] > 0:
        fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

        # Ground points
        ground = result['ground_points']
        ax1.scatter(ground[:, 0], ground[:, 1], c='gray', s=1, alpha=0.5)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title(f'Ground Points ({len(ground)})')
        ax1.set_aspect('equal')

        # Non-ground points (trail features)
        features = result['non_ground_points']
        scatter = ax2.scatter(features[:, 0], features[:, 1], c=features[:, 2],
                            cmap='viridis', s=1, alpha=0.7)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title(f'Trail Features ({len(features)})')
        ax2.set_aspect('equal')
        plt.colorbar(scatter, ax=ax2, label='Height (m)')

        plt.tight_layout()
        plt.savefig("output_segmentation.png", dpi=150, bbox_inches='tight')
        print("Saved: output_segmentation.png")

    # Plot processed result
    if result['num_final_points'] > 0:
        fig3 = plot_point_cloud_2d(result['processed_points'], "Processed Point Cloud")
        plt.savefig("output_processed.png", dpi=150, bbox_inches='tight')
        print("Saved: output_processed.png")

    print("\nProcessing complete! Check the output PNG files.")
    print("\nNote: This example uses NumPy-based processing.")
    print("For full 3D visualization, install Open3D (requires Python 3.11 or earlier).")


if __name__ == "__main__":
    main()