"""
Incremental KD-Tree (ikd-Tree) for efficient point cloud map storage

Based on the ikd-Tree algorithm from:
"ikd-Tree: An Incremental K-D Tree for Robotic Applications"
https://arxiv.org/abs/2102.10808

Key features:
- O(log n) insertion and deletion
- Incremental updates without full rebuild
- Box-wise operations for efficient map management
- Lazy rebuilding for balanced tree maintenance
"""

import numpy as np
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass, field
from collections import deque
import threading


@dataclass
class IKDNode:
    """Node in the ikd-Tree"""
    point: np.ndarray  # 3D point
    left: Optional['IKDNode'] = None
    right: Optional['IKDNode'] = None
    axis: int = 0  # Split axis (0=x, 1=y, 2=z)
    deleted: bool = False  # Lazy deletion flag
    tree_size: int = 1  # Size of subtree
    invalid_num: int = 0  # Number of deleted nodes in subtree

    # Bounding box for this subtree
    range_min: np.ndarray = field(default_factory=lambda: np.array([np.inf, np.inf, np.inf]))
    range_max: np.ndarray = field(default_factory=lambda: np.array([-np.inf, -np.inf, -np.inf]))


class IKDTree:
    """
    Incremental KD-Tree for efficient point cloud map management

    Supports:
    - Incremental insertion of points
    - Box-wise deletion (for removing old/distant points)
    - K-nearest neighbor search
    - Range search
    - Automatic rebalancing
    """

    def __init__(self, balance_ratio: float = 0.6, delete_ratio: float = 0.3,
                 box_length: float = 0.5, downsample_size: float = 0.1):
        """
        Initialize ikd-Tree

        Args:
            balance_ratio: Threshold for tree rebalancing (alpha in paper)
            delete_ratio: Threshold for lazy deletion cleanup
            box_length: Size of voxel boxes for downsampling
            downsample_size: Minimum distance between points
        """
        self.root: Optional[IKDNode] = None
        self.balance_ratio = balance_ratio
        self.delete_ratio = delete_ratio
        self.box_length = box_length
        self.downsample_size = downsample_size

        self.total_points = 0
        self.valid_points = 0

        # Thread safety
        self._lock = threading.RLock()

        # Rebuild queue for incremental operations
        self._rebuild_queue: deque = deque()

    def insert(self, point: np.ndarray) -> bool:
        """
        Insert a single point into the tree

        Args:
            point: 3D point (x, y, z)

        Returns:
            True if inserted, False if duplicate/too close
        """
        with self._lock:
            point = np.asarray(point, dtype=np.float64)

            if self.root is None:
                self.root = IKDNode(
                    point=point,
                    axis=0,
                    range_min=point.copy(),
                    range_max=point.copy()
                )
                self.total_points = 1
                self.valid_points = 1
                return True

            # Check for duplicates within downsample distance
            if self.downsample_size > 0:
                nearest, dist = self.nearest_neighbor(point)
                if nearest is not None and dist < self.downsample_size:
                    return False  # Too close to existing point

            # Insert into tree
            self._insert_recursive(self.root, point, 0)
            self.total_points += 1
            self.valid_points += 1

            # Check if rebalancing needed
            self._check_rebalance()

            return True

    def insert_points(self, points: np.ndarray) -> int:
        """
        Insert multiple points into the tree

        Args:
            points: Nx3 array of points

        Returns:
            Number of points actually inserted
        """
        inserted = 0
        for point in points:
            if self.insert(point):
                inserted += 1
        return inserted

    def _insert_recursive(self, node: IKDNode, point: np.ndarray, depth: int) -> None:
        """Recursively insert point into subtree"""
        axis = depth % 3

        # Update bounding box
        node.range_min = np.minimum(node.range_min, point)
        node.range_max = np.maximum(node.range_max, point)
        node.tree_size += 1

        if point[axis] < node.point[axis]:
            if node.left is None:
                node.left = IKDNode(
                    point=point,
                    axis=(depth + 1) % 3,
                    range_min=point.copy(),
                    range_max=point.copy()
                )
            else:
                self._insert_recursive(node.left, point, depth + 1)
        else:
            if node.right is None:
                node.right = IKDNode(
                    point=point,
                    axis=(depth + 1) % 3,
                    range_min=point.copy(),
                    range_max=point.copy()
                )
            else:
                self._insert_recursive(node.right, point, depth + 1)

    def nearest_neighbor(self, query: np.ndarray) -> Tuple[Optional[np.ndarray], float]:
        """
        Find nearest neighbor to query point

        Args:
            query: 3D query point

        Returns:
            (nearest_point, distance) or (None, inf) if tree empty
        """
        with self._lock:
            if self.root is None:
                return None, float('inf')

            query = np.asarray(query, dtype=np.float64)
            best = [None, float('inf')]  # [point, distance]

            self._nn_search(self.root, query, best, 0)

            return best[0], best[1]

    def _nn_search(self, node: Optional[IKDNode], query: np.ndarray,
                   best: list, depth: int) -> None:
        """Recursive nearest neighbor search"""
        if node is None:
            return

        if not node.deleted:
            dist = np.linalg.norm(node.point - query)
            if dist < best[1]:
                best[0] = node.point
                best[1] = dist

        axis = depth % 3
        diff = query[axis] - node.point[axis]

        # Search closer subtree first
        if diff < 0:
            first, second = node.left, node.right
        else:
            first, second = node.right, node.left

        self._nn_search(first, query, best, depth + 1)

        # Search other subtree if it could contain closer point
        if abs(diff) < best[1]:
            self._nn_search(second, query, best, depth + 1)

    def k_nearest(self, query: np.ndarray, k: int) -> List[Tuple[np.ndarray, float]]:
        """
        Find k nearest neighbors

        Args:
            query: 3D query point
            k: Number of neighbors to find

        Returns:
            List of (point, distance) tuples, sorted by distance
        """
        with self._lock:
            if self.root is None:
                return []

            query = np.asarray(query, dtype=np.float64)
            results = []  # List of (distance, point)

            self._knn_search(self.root, query, k, results, 0)

            # Sort by distance and convert to (point, distance) format
            results.sort(key=lambda x: x[0])
            return [(point, dist) for dist, point in results[:k]]

    def _knn_search(self, node: Optional[IKDNode], query: np.ndarray,
                    k: int, results: list, depth: int) -> None:
        """Recursive k-nearest neighbor search"""
        if node is None:
            return

        if not node.deleted:
            dist = np.linalg.norm(node.point - query)

            if len(results) < k:
                results.append((dist, node.point))
                results.sort(key=lambda x: x[0], reverse=True)
            elif dist < results[0][0]:
                results[0] = (dist, node.point)
                results.sort(key=lambda x: x[0], reverse=True)

        axis = depth % 3
        diff = query[axis] - node.point[axis]

        if diff < 0:
            first, second = node.left, node.right
        else:
            first, second = node.right, node.left

        self._knn_search(first, query, k, results, depth + 1)

        # Check if we need to search other subtree
        max_dist = results[0][0] if len(results) == k else float('inf')
        if abs(diff) < max_dist:
            self._knn_search(second, query, k, results, depth + 1)

    def range_search(self, center: np.ndarray, radius: float) -> List[np.ndarray]:
        """
        Find all points within radius of center

        Args:
            center: 3D center point
            radius: Search radius

        Returns:
            List of points within radius
        """
        with self._lock:
            if self.root is None:
                return []

            center = np.asarray(center, dtype=np.float64)
            results = []

            self._range_search(self.root, center, radius, results, 0)

            return results

    def _range_search(self, node: Optional[IKDNode], center: np.ndarray,
                      radius: float, results: list, depth: int) -> None:
        """Recursive range search"""
        if node is None:
            return

        # Check if subtree bounding box intersects search sphere
        if not self._box_intersects_sphere(node.range_min, node.range_max, center, radius):
            return

        if not node.deleted:
            dist = np.linalg.norm(node.point - center)
            if dist <= radius:
                results.append(node.point)

        self._range_search(node.left, center, radius, results, depth + 1)
        self._range_search(node.right, center, radius, results, depth + 1)

    def _box_intersects_sphere(self, box_min: np.ndarray, box_max: np.ndarray,
                               center: np.ndarray, radius: float) -> bool:
        """Check if axis-aligned box intersects sphere"""
        # Find closest point on box to sphere center
        closest = np.clip(center, box_min, box_max)
        dist = np.linalg.norm(closest - center)
        return dist <= radius

    def delete_by_box(self, box_min: np.ndarray, box_max: np.ndarray) -> int:
        """
        Delete all points within a bounding box (lazy deletion)

        Args:
            box_min: Minimum corner of box
            box_max: Maximum corner of box

        Returns:
            Number of points deleted
        """
        with self._lock:
            if self.root is None:
                return 0

            box_min = np.asarray(box_min, dtype=np.float64)
            box_max = np.asarray(box_max, dtype=np.float64)

            deleted = self._delete_by_box(self.root, box_min, box_max)
            self.valid_points -= deleted

            # Check if cleanup needed
            if self.total_points > 0:
                delete_ratio = (self.total_points - self.valid_points) / self.total_points
                if delete_ratio > self.delete_ratio:
                    self._cleanup_deleted()

            return deleted

    def _delete_by_box(self, node: Optional[IKDNode], box_min: np.ndarray,
                       box_max: np.ndarray) -> int:
        """Recursively delete points in box"""
        if node is None:
            return 0

        # Check if subtree bounding box intersects deletion box
        if not self._boxes_intersect(node.range_min, node.range_max, box_min, box_max):
            return 0

        deleted = 0

        if not node.deleted:
            if self._point_in_box(node.point, box_min, box_max):
                node.deleted = True
                node.invalid_num += 1
                deleted = 1

        deleted += self._delete_by_box(node.left, box_min, box_max)
        deleted += self._delete_by_box(node.right, box_min, box_max)

        # Update invalid count
        node.invalid_num = (
            (1 if node.deleted else 0) +
            (node.left.invalid_num if node.left else 0) +
            (node.right.invalid_num if node.right else 0)
        )

        return deleted

    def _boxes_intersect(self, box1_min: np.ndarray, box1_max: np.ndarray,
                         box2_min: np.ndarray, box2_max: np.ndarray) -> bool:
        """Check if two axis-aligned boxes intersect"""
        return np.all(box1_max >= box2_min) and np.all(box2_max >= box1_min)

    def _point_in_box(self, point: np.ndarray, box_min: np.ndarray,
                      box_max: np.ndarray) -> bool:
        """Check if point is inside box"""
        return np.all(point >= box_min) and np.all(point <= box_max)

    def _check_rebalance(self) -> None:
        """Check if tree needs rebalancing"""
        if self.root is None:
            return

        # Check balance ratio
        left_size = self.root.left.tree_size if self.root.left else 0
        right_size = self.root.right.tree_size if self.root.right else 0
        total = left_size + right_size + 1

        if total > 100:  # Only rebalance larger trees
            ratio = max(left_size, right_size) / total
            if ratio > self.balance_ratio:
                self._rebuild_tree()

    def _rebuild_tree(self) -> None:
        """Rebuild tree from valid points for better balance"""
        points = self.get_all_points()
        if len(points) == 0:
            self.root = None
            return

        # Build balanced tree
        self.root = self._build_balanced(points, 0)
        self.total_points = len(points)
        self.valid_points = len(points)

    def _build_balanced(self, points: np.ndarray, depth: int) -> Optional[IKDNode]:
        """Build balanced tree from sorted points"""
        if len(points) == 0:
            return None

        axis = depth % 3

        # Sort by axis and find median
        sorted_indices = np.argsort(points[:, axis])
        median_idx = len(sorted_indices) // 2

        median_point = points[sorted_indices[median_idx]]

        node = IKDNode(
            point=median_point,
            axis=axis,
            range_min=points.min(axis=0),
            range_max=points.max(axis=0),
            tree_size=len(points)
        )

        left_points = points[sorted_indices[:median_idx]]
        right_points = points[sorted_indices[median_idx + 1:]]

        node.left = self._build_balanced(left_points, depth + 1)
        node.right = self._build_balanced(right_points, depth + 1)

        return node

    def _cleanup_deleted(self) -> None:
        """Remove deleted nodes by rebuilding tree"""
        self._rebuild_tree()

    def get_all_points(self) -> np.ndarray:
        """
        Get all valid (non-deleted) points

        Returns:
            Nx3 array of points
        """
        with self._lock:
            if self.root is None:
                return np.array([]).reshape(0, 3)

            points = []
            self._collect_points(self.root, points)

            if len(points) == 0:
                return np.array([]).reshape(0, 3)

            return np.array(points)

    def _collect_points(self, node: Optional[IKDNode], points: list) -> None:
        """Collect all valid points from subtree"""
        if node is None:
            return

        if not node.deleted:
            points.append(node.point)

        self._collect_points(node.left, points)
        self._collect_points(node.right, points)

    def size(self) -> int:
        """Get number of valid points"""
        return self.valid_points

    def clear(self) -> None:
        """Clear all points from tree"""
        with self._lock:
            self.root = None
            self.total_points = 0
            self.valid_points = 0


if __name__ == "__main__":
    # Demo usage
    print("=== ikd-Tree Demo ===\n")

    # Create tree
    tree = IKDTree(downsample_size=0.1)

    # Insert random points
    np.random.seed(42)
    points = np.random.randn(1000, 3) * 10

    print(f"Inserting {len(points)} points...")
    inserted = tree.insert_points(points)
    print(f"  Inserted: {inserted} (others filtered by downsampling)")
    print(f"  Tree size: {tree.size()}")

    # Nearest neighbor search
    query = np.array([0.0, 0.0, 0.0])
    nearest, dist = tree.nearest_neighbor(query)
    print(f"\nNearest to origin: {nearest}, distance: {dist:.3f}")

    # K-nearest search
    k = 5
    knn = tree.k_nearest(query, k)
    print(f"\n{k}-nearest neighbors:")
    for i, (point, d) in enumerate(knn):
        print(f"  {i+1}. {point} (dist: {d:.3f})")

    # Range search
    radius = 2.0
    in_range = tree.range_search(query, radius)
    print(f"\nPoints within {radius}m of origin: {len(in_range)}")

    # Box deletion
    box_min = np.array([-5, -5, -5])
    box_max = np.array([5, 5, 5])
    deleted = tree.delete_by_box(box_min, box_max)
    print(f"\nDeleted {deleted} points in box [-5,5]^3")
    print(f"  Tree size after deletion: {tree.size()}")