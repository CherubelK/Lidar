"""
Demo: Change Detection, Material Classification, Occupancy Detection

Exercises the three new src/data_processing modules built on top of data the
L2 already captures but the existing pipeline never used downstream
(reflectivity intensity) or never computed (geometric diffs between scans,
transient-object tracking).

Uses synthetic point clouds so the demo is deterministic and runnable
without hardware -- swap in real saved scans (e.g. from data/complete_slam/)
by loading the *_map.npy arrays in place of the generated ones below.
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.data_processing import ChangeDetector, MaterialClassifier, OccupancyDetector


def make_room(x_range=(0, 6), y_range=(0, 4), z_range=(0, 2.5), n_wall_points=4000, seed=0):
    """Generate a simple box-room point cloud (floor + 4 walls) as a stand-in for a SLAM map."""
    rng = np.random.default_rng(seed)
    points = []

    # Floor
    x = rng.uniform(*x_range, n_wall_points // 2)
    y = rng.uniform(*y_range, n_wall_points // 2)
    z = np.full_like(x, z_range[0])
    points.append(np.stack([x, y, z], axis=1))

    # Four walls
    for axis, fixed_val in [(0, x_range[0]), (0, x_range[1]), (1, y_range[0]), (1, y_range[1])]:
        n = n_wall_points // 8
        a = rng.uniform(*(y_range if axis == 0 else x_range), n)
        z = rng.uniform(*z_range, n)
        fixed = np.full(n, fixed_val)
        if axis == 0:
            wall = np.stack([fixed, a, z], axis=1)
        else:
            wall = np.stack([a, fixed, z], axis=1)
        points.append(wall)

    return np.vstack(points).astype(np.float32)


def main():
    print("=" * 70)
    print("NEW CAPABILITIES DEMO: Change Detection / Materials / Occupancy")
    print("=" * 70)

    # ------------------------------------------------------------------
    # 1. CHANGE DETECTION
    # ------------------------------------------------------------------
    print("\n--- 1. Change Detection ---")
    baseline = make_room(seed=1)

    # "Current" scan: same room, but a pallet-sized box appeared near (1,1)
    # and a previously-present box near (4,3) is now gone.
    rng = np.random.default_rng(2)
    added_box = rng.uniform([0.8, 0.8, 0.0], [1.4, 1.4, 1.0], size=(600, 3)).astype(np.float32)
    combined = np.vstack([baseline, added_box])
    is_box_point = np.zeros(len(combined), dtype=bool)
    is_box_point[len(baseline):] = True

    keep_mask = ~(
        (np.abs(combined[:, 0] - 4.0) < 0.4) & (np.abs(combined[:, 1] - 3.0) < 0.4)
    )
    current = combined[keep_mask]
    is_box_point = is_box_point[keep_mask]

    detector = ChangeDetector(voxel_size=0.05, cluster_eps=0.15, cluster_min_points=5)
    report = detector.detect(baseline, current, align=True)

    print(f"Alignment fitness: {report.fitness:.2f}")
    print(f"Added voxels: {len(report.added_points)} | Removed voxels: {len(report.removed_points)} "
          f"| Persistent voxels: {len(report.persistent_points)}")
    print(f"Change clusters found: {len(report.clusters)}")
    for cluster in report.clusters:
        print(f"  - {cluster.change_type:8s} cluster at {cluster.centroid.round(2)} "
              f"({cluster.num_points} pts)")

    # ------------------------------------------------------------------
    # 2. MATERIAL CLASSIFICATION
    # ------------------------------------------------------------------
    print("\n--- 2. Material Classification ---")
    rng = np.random.default_rng(3)
    n = len(current)
    distances = np.linalg.norm(current, axis=1)

    # Synthetic reflectivity: floor/walls = medium (drywall-like), the
    # added box = high (assume it's a metal pallet cage). Intensity falls
    # off with distance like a real diffuse return before normalization.
    base_reflectivity = np.full(n, 0.3)
    base_reflectivity[is_box_point] = 0.7
    raw_intensity = np.clip(base_reflectivity / np.clip(distances, 0.5, None) ** 2, 0, 1)
    raw_intensity = np.clip(raw_intensity + rng.normal(0, 0.02, n), 0, 1)

    classifier = MaterialClassifier()
    bands = classifier.classify(current, raw_intensity, sensor_origin=np.array([3.0, 2.0, 1.2]))
    for name, pts in bands.items():
        print(f"  {name:24s}: {len(pts)} points")

    # ------------------------------------------------------------------
    # 3. OCCUPANCY DETECTION
    # ------------------------------------------------------------------
    print("\n--- 3. Occupancy Detection ---")
    occupancy = OccupancyDetector(baseline_points=baseline, occupancy_threshold=0.15,
                                   cluster_eps=0.3, cluster_min_points=8, max_match_distance=1.0)

    # Simulate a person walking across the room over 5 frames.
    rng = np.random.default_rng(4)
    for frame_idx in range(5):
        t = float(frame_idx)
        person_pos = np.array([1.0 + frame_idx * 0.8, 2.0, 0.0])
        person_points = rng.normal(person_pos, 0.1, size=(40, 3))
        person_points[:, 2] = rng.uniform(0.0, 1.7, 40)  # person height
        frame = np.vstack([baseline[::20], person_points])  # sparse baseline + person

        tracks = occupancy.process_frame(frame, timestamp=t)
        print(f"  Frame {frame_idx} (t={t:.0f}s): {len(tracks)} active occupant(s), "
              f"ids={[tr.track_id for tr in tracks]}")

    print(f"\nCurrent occupancy: {occupancy.current_occupancy}")
    print(f"Total foot traffic (cumulative distinct occupants): {occupancy.total_foot_traffic}")

    print("\n" + "=" * 70)
    print("Demo complete.")


if __name__ == "__main__":
    main()
