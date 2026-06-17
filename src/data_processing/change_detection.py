"""
Change Detection Between Repeated Scans
Aligns two point clouds of the same space captured at different times and
diffs them to find what was added, removed, or persisted -- the "3D + time"
capability needed for warehouse inventory tracking and construction
progress monitoring.
"""
import numpy as np
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN
from typing import List
from dataclasses import dataclass
import logging

from .icp_registration import ICPRegistration

logger = logging.getLogger(__name__)


@dataclass
class ChangeCluster:
    """A spatially-contiguous group of added or removed points."""
    points: np.ndarray
    centroid: np.ndarray
    change_type: str  # 'added' or 'removed'
    num_points: int


@dataclass
class ChangeReport:
    transformation: np.ndarray
    fitness: float
    added_points: np.ndarray
    removed_points: np.ndarray
    persistent_points: np.ndarray
    clusters: List[ChangeCluster]


class ChangeDetector:
    """
    Detects changes between two scans of the same space (e.g. a warehouse
    aisle scanned this week vs. last week, or a job site before/after a
    construction phase).

    Pipeline:
    1. Register `current` onto `baseline` with ICP so both clouds share a
       coordinate frame, even if the sensor's starting pose drifted between
       sessions.
    2. Voxel-downsample both clouds, then classify each downsampled point
       as added/removed/persistent by nearest-neighbor distance against
       the other cloud. Distance-based comparison (rather than exact voxel
       bucket equality) is what makes this tolerant of the few centimeters
       of residual misalignment ICP typically leaves behind -- an exact
       bucket-equality diff flips almost every point near a grid boundary
       from a sub-voxel rotation, which makes it useless in practice.
    3. Cluster the added/removed points into discrete change regions (e.g.
       "a pallet appeared here") instead of a noisy per-point list.
    """

    def __init__(self, voxel_size: float = 0.05, match_tolerance: float = None,
                 cluster_eps: float = 0.15, cluster_min_points: int = 5):
        """
        Args:
            voxel_size: downsampling resolution (m) applied to both clouds
                before comparison
            match_tolerance: max distance (m) for a point to be considered
                "explained" by the other cloud. Defaults to 2x voxel_size to
                absorb residual ICP alignment error; tighten it once you've
                verified registration quality on real data.
            cluster_eps: DBSCAN radius (m) for grouping changed points into
                discrete regions
            cluster_min_points: minimum points for a cluster to count as a
                real change region rather than noise
        """
        self.voxel_size = voxel_size
        self.match_tolerance = match_tolerance if match_tolerance is not None else voxel_size * 2
        self.cluster_eps = cluster_eps
        self.cluster_min_points = cluster_min_points
        self.icp = ICPRegistration(max_correspondence_distance=0.5)

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """Replace each occupied voxel with the centroid of points inside it."""
        if len(points) == 0:
            return points

        voxel_idx = np.floor(points / self.voxel_size).astype(np.int64)
        buckets: dict = {}
        for idx, pt in zip(map(tuple, voxel_idx), points):
            buckets.setdefault(idx, []).append(pt)

        return np.array([np.mean(pts, axis=0) for pts in buckets.values()])

    def detect(self, baseline_points: np.ndarray, current_points: np.ndarray,
               align: bool = True) -> ChangeReport:
        """
        Compare a current scan against a baseline scan of the same space.

        Args:
            baseline_points: Nx3 reference scan (e.g. last week's scan)
            current_points: Mx3 new scan (e.g. today's scan)
            align: whether to ICP-register current onto baseline first.
                Set False if both clouds are already in a shared SLAM world
                frame (e.g. both came from the same pose-graph map).

        Returns:
            ChangeReport with added/removed/persistent points and clusters.
        """
        transformation = np.eye(4)
        fitness = 1.0
        aligned_current = current_points

        if align:
            transformation, fitness = self.icp.register(current_points, baseline_points)
            homogeneous = np.hstack([current_points, np.ones((len(current_points), 1))])
            aligned_current = (transformation @ homogeneous.T).T[:, :3]
            logger.info(f"Change detection alignment fitness: {fitness:.2f}")

        baseline_down = self._voxel_downsample(baseline_points)
        current_down = self._voxel_downsample(aligned_current)

        if len(baseline_down) == 0 or len(current_down) == 0:
            empty = np.empty((0, 3))
            return ChangeReport(transformation, fitness, empty, empty, empty, [])

        baseline_tree = cKDTree(baseline_down)
        current_tree = cKDTree(current_down)

        # A current point with no nearby baseline point is new -> added.
        dist_to_baseline, _ = baseline_tree.query(current_down, k=1)
        added_points = current_down[dist_to_baseline > self.match_tolerance]

        # A baseline point with no nearby current point is gone -> removed.
        dist_to_current, _ = current_tree.query(baseline_down, k=1)
        removed_mask = dist_to_current > self.match_tolerance
        removed_points = baseline_down[removed_mask]
        persistent_points = baseline_down[~removed_mask]

        clusters = self._cluster(added_points, 'added') + self._cluster(removed_points, 'removed')

        logger.info(
            f"Change detection: {len(added_points)} added voxels, "
            f"{len(removed_points)} removed voxels, "
            f"{len(persistent_points)} persistent voxels, "
            f"{len(clusters)} change clusters"
        )

        return ChangeReport(
            transformation=transformation,
            fitness=fitness,
            added_points=added_points,
            removed_points=removed_points,
            persistent_points=persistent_points,
            clusters=clusters,
        )

    def _cluster(self, points: np.ndarray, change_type: str) -> List[ChangeCluster]:
        """Group nearby added/removed voxels into discrete change regions."""
        if len(points) < self.cluster_min_points:
            return []

        labels = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_points).fit_predict(points)

        clusters = []
        for label in set(labels):
            if label == -1:
                continue  # noise, not a real change region
            cluster_points = points[labels == label]
            clusters.append(ChangeCluster(
                points=cluster_points,
                centroid=cluster_points.mean(axis=0),
                change_type=change_type,
                num_points=len(cluster_points),
            ))
        return clusters
