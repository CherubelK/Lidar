"""
Privacy-Preserving Occupancy Detection
Detects and tracks transient objects (people, forklifts, moved equipment)
against a known-static baseline map using geometry only -- no camera, so it
sidesteps the privacy concerns that come with vision-based occupancy
counting in employee work areas.

A point in a live frame that doesn't match anything in the static baseline
map is "transient" -- either something temporarily occupying otherwise-empty
space, or a moving occluder. Clustering transient points per frame and
matching clusters across frames (by nearest centroid within a max travel
distance) gives simple multi-object tracking without needing a full
tracking framework.
"""
import numpy as np
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN
from dataclasses import dataclass
from typing import Dict, List, Optional
import logging

logger = logging.getLogger(__name__)


@dataclass
class TrackedOccupant:
    track_id: int
    centroid: np.ndarray
    num_points: int
    first_seen: float
    last_seen: float
    frames_seen: int = 1


class OccupancyDetector:
    """
    Tracks transient occupants against a static baseline map across a
    sequence of live frames.
    """

    def __init__(self, baseline_points: np.ndarray,
                 occupancy_threshold: float = 0.15,
                 cluster_eps: float = 0.3,
                 cluster_min_points: int = 8,
                 max_match_distance: float = 0.8,
                 track_timeout: float = 2.0):
        """
        Args:
            baseline_points: Nx3 static map of the empty space (e.g. a
                warehouse aisle SLAM map captured with nothing in the aisle)
            occupancy_threshold: min distance (m) from the nearest baseline
                point for a live point to count as transient, not sensor
                noise around an existing static surface
            cluster_eps: DBSCAN radius (m) for grouping transient points
                into discrete occupants
            cluster_min_points: minimum points for a cluster to count as
                a real occupant rather than noise
            max_match_distance: max centroid travel (m) between consecutive
                frames for a cluster to be considered the same occupant
            track_timeout: seconds after which an unseen track is dropped
        """
        self.baseline_tree = cKDTree(baseline_points)
        self.occupancy_threshold = occupancy_threshold
        self.cluster_eps = cluster_eps
        self.cluster_min_points = cluster_min_points
        self.max_match_distance = max_match_distance
        self.track_timeout = track_timeout

        self._tracks: Dict[int, TrackedOccupant] = {}
        self._next_track_id = 0
        self._total_tracks_seen = 0

    def detect_transient_points(self, frame_points: np.ndarray) -> np.ndarray:
        """Return the subset of frame_points not explained by the baseline map."""
        if len(frame_points) == 0:
            return np.empty((0, 3))
        distances, _ = self.baseline_tree.query(frame_points, k=1)
        return frame_points[distances > self.occupancy_threshold]

    def process_frame(self, frame_points: np.ndarray, timestamp: float) -> List[TrackedOccupant]:
        """
        Process one live frame: detect transient points, cluster them into
        candidate occupants, and update tracks.

        Returns:
            List of currently active TrackedOccupant instances.
        """
        transient = self.detect_transient_points(frame_points)
        clusters = self._cluster_centroids(transient)

        self._update_tracks(clusters, timestamp)
        self._expire_tracks(timestamp)

        return list(self._tracks.values())

    def _cluster_centroids(self, points: np.ndarray) -> List[tuple]:
        """Cluster transient points, returning (centroid, num_points) pairs."""
        if len(points) < self.cluster_min_points:
            return []

        labels = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_points).fit_predict(points)

        clusters = []
        for label in set(labels):
            if label == -1:
                continue
            cluster_points = points[labels == label]
            clusters.append((cluster_points.mean(axis=0), len(cluster_points)))
        return clusters

    def _update_tracks(self, clusters: List[tuple], timestamp: float) -> None:
        """Greedy nearest-centroid matching of new clusters to existing tracks."""
        unmatched_track_ids = set(self._tracks.keys())
        unmatched_clusters = list(range(len(clusters)))

        # Build all (track_id, cluster_idx, distance) pairs within range, sorted nearest-first
        candidates = []
        for track_id in unmatched_track_ids:
            track = self._tracks[track_id]
            for cluster_idx in unmatched_clusters:
                centroid, _ = clusters[cluster_idx]
                dist = np.linalg.norm(centroid - track.centroid)
                if dist <= self.max_match_distance:
                    candidates.append((dist, track_id, cluster_idx))
        candidates.sort(key=lambda c: c[0])

        matched_tracks = set()
        matched_clusters = set()
        for dist, track_id, cluster_idx in candidates:
            if track_id in matched_tracks or cluster_idx in matched_clusters:
                continue
            centroid, num_points = clusters[cluster_idx]
            track = self._tracks[track_id]
            track.centroid = centroid
            track.num_points = num_points
            track.last_seen = timestamp
            track.frames_seen += 1
            matched_tracks.add(track_id)
            matched_clusters.add(cluster_idx)

        # Any cluster that didn't match an existing track becomes a new occupant
        for cluster_idx, (centroid, num_points) in enumerate(clusters):
            if cluster_idx in matched_clusters:
                continue
            track_id = self._next_track_id
            self._next_track_id += 1
            self._total_tracks_seen += 1
            self._tracks[track_id] = TrackedOccupant(
                track_id=track_id,
                centroid=centroid,
                num_points=num_points,
                first_seen=timestamp,
                last_seen=timestamp,
            )

    def _expire_tracks(self, timestamp: float) -> None:
        """Drop tracks that haven't been seen within track_timeout."""
        expired = [
            track_id for track_id, track in self._tracks.items()
            if timestamp - track.last_seen > self.track_timeout
        ]
        for track_id in expired:
            del self._tracks[track_id]

    @property
    def current_occupancy(self) -> int:
        """Number of currently active tracked occupants."""
        return len(self._tracks)

    @property
    def total_foot_traffic(self) -> int:
        """Total distinct occupants ever tracked (cumulative, not concurrent)."""
        return self._total_tracks_seen
