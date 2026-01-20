"""
Room Layout Extractor

Analyzes LiDAR point cloud data to:
1. Extract floor/ceiling to determine room boundaries
2. Create 2D projection of walls
3. Detect room outline using convex hull or alpha shape
4. Calculate room dimensions (length, width, area)
5. Generate a 2D floor plan visualization

Usage:
    python room_layout.py --scan room_scan_20260119_202824.json
    python room_layout.py --scan room_scan_20260119_202824.json --output floor_plan.png
"""

import sys
from pathlib import Path
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.collections import PatchCollection
import argparse

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))


def load_scan(filename):
    """Load point cloud from JSON file."""
    scan_path = Path("web/models") / filename
    if not scan_path.exists():
        # Try absolute path
        scan_path = Path(filename)

    if not scan_path.exists():
        raise FileNotFoundError(f"Scan not found: {filename}")

    with open(scan_path, 'r') as f:
        data = json.load(f)

    vertices = np.array(data['vertices']).reshape(-1, 3)
    return vertices


def extract_wall_slice(points, z_min=None, z_max=None, num_slices=5):
    """
    Extract multiple horizontal slices through the point cloud
    to capture wall points at different heights.

    Args:
        points: Nx3 point cloud
        z_min, z_max: Height range (auto-detected if None)
        num_slices: Number of horizontal slices to combine

    Returns:
        2D points (x, y) from all slices combined
    """
    z_values = points[:, 2]

    if z_min is None:
        z_min = np.percentile(z_values, 5)  # 5th percentile to avoid outliers
    if z_max is None:
        z_max = np.percentile(z_values, 95)  # 95th percentile

    # Create multiple slices and combine
    all_wall_points = []
    slice_thickness = 0.1  # 10cm slices

    # Sample at different heights (skip very bottom and top)
    heights = np.linspace(z_min + 0.3, z_max - 0.3, num_slices)

    for h in heights:
        mask = (z_values >= h - slice_thickness/2) & (z_values <= h + slice_thickness/2)
        slice_points = points[mask][:, :2]
        all_wall_points.append(slice_points)

    combined = np.vstack(all_wall_points) if all_wall_points else np.array([])
    return combined, z_min, z_max


def grid_based_boundary(points, resolution=0.05, min_points=2):
    """
    Create boundary using a grid-based approach.
    More robust than alpha shapes for noisy data.

    1. Create occupancy grid
    2. Find outer boundary cells
    3. Extract boundary polygon
    """
    if len(points) < 10:
        return None

    # Create grid
    x_min, y_min = points.min(axis=0) - resolution
    x_max, y_max = points.max(axis=0) + resolution

    nx = int((x_max - x_min) / resolution) + 1
    ny = int((y_max - y_min) / resolution) + 1

    # Count points per cell
    grid = np.zeros((nx, ny), dtype=int)

    for p in points:
        ix = int((p[0] - x_min) / resolution)
        iy = int((p[1] - y_min) / resolution)
        ix = max(0, min(nx-1, ix))
        iy = max(0, min(ny-1, iy))
        grid[ix, iy] += 1

    # Mark occupied cells
    occupied = grid >= min_points

    # Find boundary cells (occupied with at least one empty neighbor)
    boundary_mask = np.zeros_like(occupied)
    for i in range(1, nx-1):
        for j in range(1, ny-1):
            if occupied[i, j]:
                # Check 8-neighbors
                neighbors = occupied[i-1:i+2, j-1:j+2]
                if not neighbors.all():  # Has at least one empty neighbor
                    boundary_mask[i, j] = True

    # Also check edges
    boundary_mask[0, :] |= occupied[0, :]
    boundary_mask[-1, :] |= occupied[-1, :]
    boundary_mask[:, 0] |= occupied[:, 0]
    boundary_mask[:, -1] |= occupied[:, -1]

    # Extract boundary points
    boundary_indices = np.where(boundary_mask)
    boundary_points = np.column_stack([
        boundary_indices[0] * resolution + x_min + resolution/2,
        boundary_indices[1] * resolution + y_min + resolution/2
    ])

    return boundary_points, occupied, (x_min, y_min, resolution, nx, ny)


def convex_hull_boundary(points):
    """Simple convex hull boundary."""
    from scipy.spatial import ConvexHull

    if len(points) < 3:
        return None

    try:
        hull = ConvexHull(points)
        return points[hull.vertices]
    except:
        return None


def fit_rectangle(points, angle_step=1):
    """
    Fit minimum area bounding rectangle to points.
    Returns corners of the rectangle.
    """
    if len(points) < 3:
        return None, None

    from scipy.spatial import ConvexHull

    try:
        hull = ConvexHull(points)
        hull_points = points[hull.vertices]
    except:
        hull_points = points

    min_area = float('inf')
    best_rect = None
    best_angle = 0

    # Try many angles
    for angle_deg in range(0, 180, angle_step):
        angle = np.radians(angle_deg)
        cos_a, sin_a = np.cos(-angle), np.sin(-angle)

        # Rotate points
        rotated = np.column_stack([
            hull_points[:, 0] * cos_a - hull_points[:, 1] * sin_a,
            hull_points[:, 0] * sin_a + hull_points[:, 1] * cos_a
        ])

        # Axis-aligned bounding box
        min_xy = rotated.min(axis=0)
        max_xy = rotated.max(axis=0)

        width = max_xy[0] - min_xy[0]
        height = max_xy[1] - min_xy[1]
        area = width * height

        if area < min_area:
            min_area = area
            # Rectangle corners in rotated space
            corners_rot = np.array([
                [min_xy[0], min_xy[1]],
                [max_xy[0], min_xy[1]],
                [max_xy[0], max_xy[1]],
                [min_xy[0], max_xy[1]]
            ])
            # Rotate back
            cos_a, sin_a = np.cos(angle), np.sin(angle)
            best_rect = np.column_stack([
                corners_rot[:, 0] * cos_a - corners_rot[:, 1] * sin_a,
                corners_rot[:, 0] * sin_a + corners_rot[:, 1] * cos_a
            ])
            best_angle = angle_deg

    return best_rect, best_angle


def detect_dominant_directions(points, num_bins=180):
    """
    Detect dominant wall directions using Hough-like voting.
    """
    from scipy.spatial import ConvexHull

    try:
        hull = ConvexHull(points)
        hull_points = points[hull.vertices]
    except:
        hull_points = points

    # Compute edge directions
    angles = []
    weights = []
    n = len(hull_points)

    for i in range(n):
        p1 = hull_points[i]
        p2 = hull_points[(i + 1) % n]
        vec = p2 - p1
        length = np.linalg.norm(vec)
        if length > 0.1:
            angle = np.degrees(np.arctan2(vec[1], vec[0])) % 180
            angles.append(angle)
            weights.append(length)

    if not angles:
        return [0, 90]

    # Histogram voting
    hist, bin_edges = np.histogram(angles, bins=num_bins, range=(0, 180), weights=weights)

    # Find peaks (dominant directions)
    peaks = []
    for i in range(len(hist)):
        if hist[i] > 0:
            # Local maximum check
            left = hist[(i - 1) % len(hist)]
            right = hist[(i + 1) % len(hist)]
            if hist[i] >= left and hist[i] >= right:
                peaks.append((hist[i], bin_edges[i]))

    peaks.sort(reverse=True)

    # Return top 2 perpendicular directions
    if len(peaks) >= 1:
        dir1 = peaks[0][1]
        dir2 = (dir1 + 90) % 180
        return [dir1, dir2]

    return [0, 90]


def fit_walls_to_directions(boundary_points, directions, tolerance=0.2):
    """
    Fit straight wall segments aligned with dominant directions.
    """
    walls = []

    for direction in directions:
        angle = np.radians(direction)

        # Project points onto this direction
        cos_a, sin_a = np.cos(-angle), np.sin(-angle)
        rotated = np.column_stack([
            boundary_points[:, 0] * cos_a - boundary_points[:, 1] * sin_a,
            boundary_points[:, 0] * sin_a + boundary_points[:, 1] * cos_a
        ])

        # Find clusters along the perpendicular axis (these are wall lines)
        perp_coords = rotated[:, 1]

        # Simple clustering by binning
        bin_width = tolerance
        bins = np.floor(perp_coords / bin_width).astype(int)
        unique_bins = np.unique(bins)

        for b in unique_bins:
            mask = bins == b
            cluster_points = rotated[mask]

            if len(cluster_points) < 3:
                continue

            # Wall line at this perpendicular position
            perp_pos = cluster_points[:, 1].mean()
            along_min = cluster_points[:, 0].min()
            along_max = cluster_points[:, 0].max()
            length = along_max - along_min

            if length < 0.5:  # Skip very short segments
                continue

            # Convert back to original coordinates
            cos_a, sin_a = np.cos(angle), np.sin(angle)
            start = np.array([
                along_min * cos_a - perp_pos * sin_a,
                along_min * sin_a + perp_pos * cos_a
            ])
            end = np.array([
                along_max * cos_a - perp_pos * sin_a,
                along_max * sin_a + perp_pos * cos_a
            ])

            walls.append({
                'start': start,
                'end': end,
                'length': length,
                'angle': direction,
                'midpoint': (start + end) / 2
            })

    return walls


def compute_room_dimensions(boundary, rect=None):
    """
    Compute room dimensions.
    """
    if rect is not None and len(rect) == 4:
        # Use fitted rectangle
        side1 = np.linalg.norm(rect[1] - rect[0])
        side2 = np.linalg.norm(rect[2] - rect[1])
        width = min(side1, side2)
        length = max(side1, side2)
        area = width * length
    else:
        # Use bounding box
        x_range = boundary[:, 0].max() - boundary[:, 0].min()
        y_range = boundary[:, 1].max() - boundary[:, 1].min()
        width = min(x_range, y_range)
        length = max(x_range, y_range)
        area = polygon_area(boundary) if len(boundary) >= 3 else width * length

    return width, length, area


def polygon_area(vertices):
    """Compute area of polygon using shoelace formula."""
    n = len(vertices)
    if n < 3:
        return 0
    area = 0
    for i in range(n):
        j = (i + 1) % n
        area += vertices[i, 0] * vertices[j, 1]
        area -= vertices[j, 0] * vertices[i, 1]
    return abs(area) / 2


def generate_floor_plan(points_2d, boundary, rect, walls, dimensions, height,
                        output_path=None, grid_info=None):
    """
    Generate 2D floor plan visualization with stronger wall definition.
    All measurements in US units (feet).
    """
    # Conversion factors
    M_TO_FT = 3.28084
    M2_TO_FT2 = 10.7639

    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    width_m, length_m, area_m = dimensions
    # Convert to feet
    width_ft = width_m * M_TO_FT
    length_ft = length_m * M_TO_FT
    area_ft = area_m * M2_TO_FT2
    height_ft = height * M_TO_FT

    # Convert rectangle to feet for display
    if rect is not None:
        rect_ft = rect * M_TO_FT
    else:
        rect_ft = None

    # Convert points to feet
    points_ft = points_2d * M_TO_FT if len(points_2d) > 0 else points_2d
    boundary_ft = boundary * M_TO_FT if boundary is not None else None

    # Left plot: Point cloud with detected features
    ax1 = axes[0]
    ax1.set_title("LiDAR Point Cloud (Top View)", fontsize=14, fontweight='bold')

    if len(points_ft) > 0:
        # Subsample if needed
        if len(points_ft) > 15000:
            indices = np.random.choice(len(points_ft), 15000, replace=False)
            plot_points = points_ft[indices]
        else:
            plot_points = points_ft

        ax1.scatter(plot_points[:, 0], plot_points[:, 1],
                   s=1, c='steelblue', alpha=0.4, label='Wall Points')

    # Draw boundary points
    if boundary_ft is not None and len(boundary_ft) > 0:
        ax1.scatter(boundary_ft[:, 0], boundary_ft[:, 1],
                   s=5, c='red', alpha=0.6, label='Boundary')

    # Draw fitted rectangle
    if rect_ft is not None:
        rect_closed = np.vstack([rect_ft, rect_ft[0]])
        ax1.plot(rect_closed[:, 0], rect_closed[:, 1],
                'g-', linewidth=3, label='Fitted Room')

    ax1.set_xlabel('X (feet)', fontsize=11)
    ax1.set_ylabel('Y (feet)', fontsize=11)
    ax1.set_aspect('equal')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # Right plot: Clean floor plan
    ax2 = axes[1]
    ax2.set_title("Floor Plan", fontsize=14, fontweight='bold')
    ax2.set_facecolor('#f5f5f5')

    # Draw room outline with thick walls
    if rect_ft is not None:
        rect_closed = np.vstack([rect_ft, rect_ft[0]])

        # Fill room interior
        ax2.fill(rect_ft[:, 0], rect_ft[:, 1], color='white', zorder=1)

        # Draw thick wall lines
        wall_thickness = 4
        ax2.plot(rect_closed[:, 0], rect_closed[:, 1],
                'k-', linewidth=wall_thickness, solid_capstyle='projecting', zorder=2)

        # Add wall dimension labels
        for i in range(4):
            p1 = rect_ft[i]
            p2 = rect_ft[(i + 1) % 4]
            mid = (p1 + p2) / 2
            wall_len_ft = np.linalg.norm(p2 - p1)

            # Calculate perpendicular offset for label
            vec = p2 - p1
            perp = np.array([-vec[1], vec[0]])
            perp = perp / np.linalg.norm(perp) * 1.3  # Offset in feet

            # Determine which side to place label (outside the room)
            center = rect_ft.mean(axis=0)
            if np.dot(perp, mid - center) < 0:
                perp = -perp

            label_pos = mid + perp

            # Draw dimension line
            ax2.annotate('', xy=p1, xytext=p2,
                        arrowprops=dict(arrowstyle='<->', color='#444444', lw=1.5),
                        zorder=3)

            # Add length text in feet and inches
            feet = int(wall_len_ft)
            inches = (wall_len_ft - feet) * 12
            ax2.text(label_pos[0], label_pos[1], f"{feet}' {inches:.0f}\"",
                    ha='center', va='center', fontsize=10, fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                             edgecolor='gray', alpha=0.9), zorder=4)

    # Dimensions info box (US units only)
    width_feet = int(width_ft)
    width_inches = (width_ft - width_feet) * 12
    length_feet = int(length_ft)
    length_inches = (length_ft - length_feet) * 12
    height_feet = int(height_ft)
    height_inches = (height_ft - height_feet) * 12

    dim_text = (
        f"┌─────────────────────┐\n"
        f"│   ROOM DIMENSIONS   │\n"
        f"├─────────────────────┤\n"
        f"│ Width:              │\n"
        f"│   {width_feet}' {width_inches:.0f}\" ({width_ft:.1f} ft) │\n"
        f"│ Length:             │\n"
        f"│   {length_feet}' {length_inches:.0f}\" ({length_ft:.1f} ft)│\n"
        f"│ Ceiling:            │\n"
        f"│   {height_feet}' {height_inches:.0f}\" ({height_ft:.1f} ft) │\n"
        f"├─────────────────────┤\n"
        f"│ Floor Area:         │\n"
        f"│   {area_ft:.1f} ft²          │\n"
        f"└─────────────────────┘"
    )

    ax2.text(0.02, 0.98, dim_text, transform=ax2.transAxes,
            fontsize=10, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='square', facecolor='white', alpha=0.95,
                     edgecolor='black', linewidth=2),
            zorder=10)

    ax2.set_xlabel('X (feet)', fontsize=11)
    ax2.set_ylabel('Y (feet)', fontsize=11)
    ax2.set_aspect('equal')

    # Add scale bar (3 feet)
    add_scale_bar(ax2, length_ft=3.0)

    # Add grid
    ax2.grid(True, alpha=0.3, linestyle='--')

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=200, bbox_inches='tight',
                   facecolor='white', edgecolor='none')
        print(f"Floor plan saved to: {output_path}")

    plt.show()

    return fig


def add_scale_bar(ax, length_ft=3.0):
    """Add a scale bar to the plot (in feet)."""
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()

    # Position in lower right
    x_start = xlim[1] - length_ft - 1.0
    y_pos = ylim[0] + 0.6

    # Draw scale bar
    ax.plot([x_start, x_start + length_ft], [y_pos, y_pos], 'k-', linewidth=4, zorder=5)
    ax.plot([x_start, x_start], [y_pos - 0.25, y_pos + 0.25], 'k-', linewidth=2, zorder=5)
    ax.plot([x_start + length_ft, x_start + length_ft], [y_pos - 0.25, y_pos + 0.25], 'k-', linewidth=2, zorder=5)
    ax.text(x_start + length_ft/2, y_pos + 0.5, f"{length_ft:.0f} ft",
           ha='center', va='bottom', fontsize=10, fontweight='bold', zorder=5)


def analyze_room(scan_file, output_path=None):
    """
    Main function to analyze room from scan.
    """
    print("=" * 60)
    print("  ROOM LAYOUT ANALYZER")
    print("=" * 60)
    print(f"\nLoading scan: {scan_file}")

    # Load point cloud
    points = load_scan(scan_file)
    print(f"Loaded {len(points):,} points")

    # Get Z range
    z_min, z_max = points[:, 2].min(), points[:, 2].max()
    height = z_max - z_min
    print(f"Height range: {z_min:.2f}m to {z_max:.2f}m (total: {height:.2f}m)")

    # Extract wall slices at multiple heights
    print("\nExtracting wall points from multiple height slices...")
    wall_points, floor_z, ceiling_z = extract_wall_slice(points, num_slices=8)
    print(f"  Extracted {len(wall_points):,} wall points")

    # Downsample for processing
    if len(wall_points) > 10000:
        voxel_size = 0.03  # 3cm resolution
        voxel_indices = np.floor(wall_points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        wall_points_ds = wall_points[unique_indices]
        print(f"  Downsampled to: {len(wall_points_ds):,} points")
    else:
        wall_points_ds = wall_points

    # Grid-based boundary detection
    print("\nDetecting room boundary...")
    result = grid_based_boundary(wall_points_ds, resolution=0.08, min_points=2)

    if result is None:
        print("ERROR: Could not detect boundary, using convex hull")
        boundary = convex_hull_boundary(wall_points_ds)
    else:
        boundary, occupancy_grid, grid_info = result
        print(f"  Boundary points: {len(boundary)}")

    # Fit rectangle to room
    print("\nFitting rectangular room outline...")
    rect, angle = fit_rectangle(wall_points_ds, angle_step=1)

    if rect is not None:
        print(f"  Room orientation: {angle}°")

    # Detect dominant wall directions
    print("\nAnalyzing wall directions...")
    directions = detect_dominant_directions(wall_points_ds)
    print(f"  Dominant directions: {directions[0]:.1f}° and {directions[1]:.1f}°")

    # Compute dimensions
    width, length, area = compute_room_dimensions(boundary, rect)

    # Convert to US units
    M_TO_FT = 3.28084
    M2_TO_FT2 = 10.7639

    width_ft = width * M_TO_FT
    length_ft = length * M_TO_FT
    area_ft = area * M2_TO_FT2
    height_ft = height * M_TO_FT

    # Format as feet and inches
    def ft_to_ft_in(ft):
        feet = int(ft)
        inches = (ft - feet) * 12
        return f"{feet}' {inches:.0f}\""

    print(f"\n{'='*50}")
    print("  ROOM DIMENSIONS")
    print(f"{'='*50}")
    print(f"  Width:     {ft_to_ft_in(width_ft):>10}  ({width_ft:.1f} ft)")
    print(f"  Length:    {ft_to_ft_in(length_ft):>10}  ({length_ft:.1f} ft)")
    print(f"  Ceiling:   {ft_to_ft_in(height_ft):>10}  ({height_ft:.1f} ft)")
    print(f"  Area:      {area_ft:.1f} ft²")
    print(f"{'='*50}\n")

    # Generate floor plan
    print("Generating floor plan visualization...")
    dimensions = (width, length, area)

    generate_floor_plan(
        wall_points_ds, boundary, rect, [], dimensions, height,
        output_path=output_path
    )

    return {
        'width': width,
        'length': length,
        'area': area,
        'height': height,
        'boundary': boundary,
        'rectangle': rect
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract room layout from LiDAR scan")
    parser.add_argument('--scan', type=str, required=True,
                       help='Scan filename (in web/models/) or full path')
    parser.add_argument('--output', type=str, default=None,
                       help='Output image path (optional)')

    args = parser.parse_args()

    output = args.output
    if output is None:
        scan_name = Path(args.scan).stem
        output = f"floor_plan_{scan_name}.png"

    result = analyze_room(args.scan, output_path=output)
