"""
Pose Graph Optimization for SLAM

Implements pose graph optimization similar to g2o/GTSAM
Used to correct accumulated drift when loop closures are detected

Based on:
- g2o: A General Framework for Graph Optimization
- GTSAM: Georgia Tech Smoothing and Mapping library
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field
from scipy.optimize import minimize
from scipy.sparse import lil_matrix, csr_matrix
from scipy.sparse.linalg import spsolve
import logging

logger = logging.getLogger(__name__)


@dataclass
class PoseNode:
    """A node in the pose graph (robot pose at a specific time)"""
    id: int
    pose: np.ndarray  # 4x4 transformation matrix
    fixed: bool = False  # If True, pose won't be optimized


@dataclass
class PoseEdge:
    """An edge (constraint) between two poses"""
    id_from: int
    id_to: int
    measurement: np.ndarray  # 4x4 relative transformation
    information: np.ndarray = field(default_factory=lambda: np.eye(6))  # 6x6 information matrix

    @property
    def is_loop_closure(self) -> bool:
        """Check if this is a loop closure edge (non-sequential)"""
        return abs(self.id_to - self.id_from) > 1


def pose_to_vector(pose: np.ndarray) -> np.ndarray:
    """
    Convert 4x4 pose matrix to 6D vector [x, y, z, roll, pitch, yaw]

    Args:
        pose: 4x4 transformation matrix

    Returns:
        6D vector [x, y, z, roll, pitch, yaw]
    """
    # Translation
    x, y, z = pose[:3, 3]

    # Rotation (extract Euler angles from rotation matrix)
    R = pose[:3, :3]

    # Roll, pitch, yaw from rotation matrix
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)

    if sy > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0

    return np.array([x, y, z, roll, pitch, yaw])


def vector_to_pose(v: np.ndarray) -> np.ndarray:
    """
    Convert 6D vector to 4x4 pose matrix

    Args:
        v: 6D vector [x, y, z, roll, pitch, yaw]

    Returns:
        4x4 transformation matrix
    """
    x, y, z, roll, pitch, yaw = v

    # Rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    R = Rz @ Ry @ Rx

    pose = np.eye(4)
    pose[:3, :3] = R
    pose[:3, 3] = [x, y, z]

    return pose


def compute_edge_error(pose_from: np.ndarray, pose_to: np.ndarray,
                       measurement: np.ndarray) -> np.ndarray:
    """
    Compute error between measured and estimated relative pose

    Args:
        pose_from: 4x4 pose of first node
        pose_to: 4x4 pose of second node
        measurement: 4x4 measured relative transformation

    Returns:
        6D error vector
    """
    # Estimated relative transformation
    estimated = np.linalg.inv(pose_from) @ pose_to

    # Error in transformation
    error_matrix = np.linalg.inv(measurement) @ estimated

    # Convert to vector
    return pose_to_vector(error_matrix)


class PoseGraph:
    """
    Pose Graph for SLAM optimization

    Maintains a graph of poses (nodes) and constraints (edges)
    Optimizes all poses when loop closures are added
    """

    def __init__(self):
        """Initialize empty pose graph"""
        self.nodes: Dict[int, PoseNode] = {}
        self.edges: List[PoseEdge] = []

        # Odometry edges (sequential)
        self.odometry_edges: List[PoseEdge] = []

        # Loop closure edges
        self.loop_edges: List[PoseEdge] = []

        # Optimization parameters
        self.max_iterations = 100
        self.convergence_threshold = 1e-6

    def add_node(self, node_id: int, pose: np.ndarray, fixed: bool = False) -> None:
        """
        Add a pose node to the graph

        Args:
            node_id: Unique node identifier
            pose: 4x4 transformation matrix
            fixed: If True, pose won't change during optimization
        """
        self.nodes[node_id] = PoseNode(
            id=node_id,
            pose=pose.copy(),
            fixed=fixed
        )

    def add_edge(self, id_from: int, id_to: int, measurement: np.ndarray,
                 information: Optional[np.ndarray] = None,
                 is_loop: bool = False) -> None:
        """
        Add a constraint edge between two poses

        Args:
            id_from: Source node ID
            id_to: Target node ID
            measurement: 4x4 relative transformation
            information: 6x6 information matrix (default: identity)
            is_loop: If True, this is a loop closure constraint
        """
        if information is None:
            information = np.eye(6)

        edge = PoseEdge(
            id_from=id_from,
            id_to=id_to,
            measurement=measurement.copy(),
            information=information.copy()
        )

        self.edges.append(edge)

        if is_loop or abs(id_to - id_from) > 1:
            self.loop_edges.append(edge)
            logger.info(f"Added loop closure edge: {id_from} -> {id_to}")
        else:
            self.odometry_edges.append(edge)

    def add_odometry_edge(self, id_from: int, id_to: int,
                         relative_pose: np.ndarray) -> None:
        """
        Add sequential odometry constraint

        Args:
            id_from: Previous pose ID
            id_to: Current pose ID
            relative_pose: 4x4 relative transformation
        """
        # Higher confidence for odometry
        information = np.diag([100, 100, 100, 100, 100, 100])
        self.add_edge(id_from, id_to, relative_pose, information, is_loop=False)

    def add_loop_closure(self, id_from: int, id_to: int,
                        relative_pose: np.ndarray,
                        confidence: float = 1.0) -> None:
        """
        Add loop closure constraint

        Args:
            id_from: First pose ID (earlier in time)
            id_to: Second pose ID (later in time)
            relative_pose: 4x4 relative transformation
            confidence: Confidence in the loop closure (0-1)
        """
        # Information matrix scaled by confidence
        info_scale = 10 * confidence
        information = np.diag([info_scale] * 6)

        self.add_edge(id_from, id_to, relative_pose, information, is_loop=True)

    def optimize(self, num_iterations: Optional[int] = None) -> float:
        """
        Optimize all poses using Gauss-Newton

        Args:
            num_iterations: Maximum iterations (default: self.max_iterations)

        Returns:
            Final total error
        """
        if num_iterations is None:
            num_iterations = self.max_iterations

        if len(self.nodes) == 0 or len(self.edges) == 0:
            return 0.0

        logger.info(f"Optimizing pose graph: {len(self.nodes)} nodes, "
                   f"{len(self.edges)} edges ({len(self.loop_edges)} loops)")

        # Gauss-Newton optimization
        for iteration in range(num_iterations):
            # Build linear system
            H, b = self._build_linear_system()

            # Solve H * dx = -b
            try:
                dx = spsolve(csr_matrix(H), -b)
            except Exception as e:
                logger.warning(f"Linear solve failed: {e}")
                break

            # Check convergence
            delta_norm = np.linalg.norm(dx)
            if delta_norm < self.convergence_threshold:
                logger.info(f"Converged at iteration {iteration + 1}")
                break

            # Update poses
            self._apply_update(dx)

            if iteration % 10 == 0:
                error = self._compute_total_error()
                logger.debug(f"Iteration {iteration}: error={error:.6f}, delta={delta_norm:.6f}")

        final_error = self._compute_total_error()
        logger.info(f"Optimization complete: final error = {final_error:.6f}")

        return final_error

    def _build_linear_system(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Build the linear system H * dx = -b for Gauss-Newton

        Returns:
            (H, b) where H is the Hessian and b is the gradient
        """
        num_nodes = len(self.nodes)
        dim = 6  # 6 DOF per pose

        # Use sparse matrix for efficiency
        H = lil_matrix((num_nodes * dim, num_nodes * dim))
        b = np.zeros(num_nodes * dim)

        # Node ID to index mapping
        id_to_idx = {node_id: idx for idx, node_id in enumerate(sorted(self.nodes.keys()))}

        for edge in self.edges:
            i = id_to_idx[edge.id_from]
            j = id_to_idx[edge.id_to]

            pose_i = self.nodes[edge.id_from].pose
            pose_j = self.nodes[edge.id_to].pose

            # Compute error
            error = compute_edge_error(pose_i, pose_j, edge.measurement)

            # Jacobians (numerical approximation)
            Ji, Jj = self._compute_jacobians(pose_i, pose_j, edge.measurement)

            # Information matrix
            omega = edge.information

            # Accumulate into H and b
            # H += J^T * Omega * J
            # b += J^T * Omega * e

            if not self.nodes[edge.id_from].fixed:
                H[i*dim:(i+1)*dim, i*dim:(i+1)*dim] += Ji.T @ omega @ Ji
                H[i*dim:(i+1)*dim, j*dim:(j+1)*dim] += Ji.T @ omega @ Jj
                b[i*dim:(i+1)*dim] += Ji.T @ omega @ error

            if not self.nodes[edge.id_to].fixed:
                H[j*dim:(j+1)*dim, i*dim:(i+1)*dim] += Jj.T @ omega @ Ji
                H[j*dim:(j+1)*dim, j*dim:(j+1)*dim] += Jj.T @ omega @ Jj
                b[j*dim:(j+1)*dim] += Jj.T @ omega @ error

        # Fix first pose (anchor)
        first_node_id = min(self.nodes.keys())
        first_idx = id_to_idx[first_node_id]
        for d in range(dim):
            H[first_idx*dim + d, first_idx*dim + d] += 1e10

        return H.toarray(), b

    def _compute_jacobians(self, pose_i: np.ndarray, pose_j: np.ndarray,
                          measurement: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute Jacobians numerically

        Returns:
            (Ji, Jj) - Jacobians w.r.t. pose_i and pose_j
        """
        dim = 6
        eps = 1e-6

        Ji = np.zeros((dim, dim))
        Jj = np.zeros((dim, dim))

        e0 = compute_edge_error(pose_i, pose_j, measurement)

        # Jacobian w.r.t. pose_i
        vi = pose_to_vector(pose_i)
        for d in range(dim):
            vi_plus = vi.copy()
            vi_plus[d] += eps
            pose_i_plus = vector_to_pose(vi_plus)
            e_plus = compute_edge_error(pose_i_plus, pose_j, measurement)
            Ji[:, d] = (e_plus - e0) / eps

        # Jacobian w.r.t. pose_j
        vj = pose_to_vector(pose_j)
        for d in range(dim):
            vj_plus = vj.copy()
            vj_plus[d] += eps
            pose_j_plus = vector_to_pose(vj_plus)
            e_plus = compute_edge_error(pose_i, pose_j_plus, measurement)
            Jj[:, d] = (e_plus - e0) / eps

        return Ji, Jj

    def _apply_update(self, dx: np.ndarray) -> None:
        """Apply pose updates"""
        dim = 6
        id_to_idx = {node_id: idx for idx, node_id in enumerate(sorted(self.nodes.keys()))}

        for node_id, node in self.nodes.items():
            if node.fixed:
                continue

            idx = id_to_idx[node_id]
            delta = dx[idx*dim:(idx+1)*dim]

            # Apply update
            v = pose_to_vector(node.pose)
            v += delta
            node.pose = vector_to_pose(v)

    def _compute_total_error(self) -> float:
        """Compute total squared error"""
        total_error = 0.0

        for edge in self.edges:
            pose_i = self.nodes[edge.id_from].pose
            pose_j = self.nodes[edge.id_to].pose

            error = compute_edge_error(pose_i, pose_j, edge.measurement)
            weighted_error = error.T @ edge.information @ error
            total_error += weighted_error

        return total_error

    def get_optimized_poses(self) -> List[np.ndarray]:
        """
        Get all optimized poses in order

        Returns:
            List of 4x4 transformation matrices
        """
        sorted_ids = sorted(self.nodes.keys())
        return [self.nodes[node_id].pose.copy() for node_id in sorted_ids]

    def get_optimized_trajectory(self) -> np.ndarray:
        """
        Get optimized trajectory as Nx3 positions

        Returns:
            Nx3 array of positions
        """
        poses = self.get_optimized_poses()
        return np.array([pose[:3, 3] for pose in poses])

    def correct_point_cloud(self, points: np.ndarray, pose_idx: int) -> np.ndarray:
        """
        Transform points using optimized pose

        Args:
            points: Nx3 point cloud
            pose_idx: Index of pose to use for transformation

        Returns:
            Transformed Nx3 point cloud
        """
        sorted_ids = sorted(self.nodes.keys())
        if pose_idx >= len(sorted_ids):
            return points

        node_id = sorted_ids[pose_idx]
        pose = self.nodes[node_id].pose

        # Transform points
        points_h = np.hstack([points, np.ones((len(points), 1))])
        transformed = (pose @ points_h.T).T

        return transformed[:, :3]

    def get_stats(self) -> dict:
        """Get optimization statistics"""
        return {
            'num_nodes': len(self.nodes),
            'num_edges': len(self.edges),
            'num_odometry_edges': len(self.odometry_edges),
            'num_loop_closures': len(self.loop_edges),
            'total_error': self._compute_total_error() if len(self.edges) > 0 else 0.0
        }


if __name__ == "__main__":
    # Demo
    logging.basicConfig(level=logging.INFO)
    print("=== Pose Graph Optimization Demo ===\n")

    # Create pose graph
    graph = PoseGraph()

    # Add poses (simulating a loop)
    # Robot moves in a square and returns to start
    poses = [
        np.eye(4),  # Start at origin
    ]

    # Move forward 10m
    for i in range(1, 11):
        pose = np.eye(4)
        pose[0, 3] = i  # x = i
        poses.append(pose)

    # Turn right and move 10m
    for i in range(1, 11):
        pose = np.eye(4)
        pose[0, 3] = 10  # x = 10
        pose[1, 3] = i   # y = i
        poses.append(pose)

    # Turn right and move back 10m
    for i in range(1, 11):
        pose = np.eye(4)
        pose[0, 3] = 10 - i  # x decreasing
        pose[1, 3] = 10      # y = 10
        poses.append(pose)

    # Turn right and return to start
    for i in range(1, 11):
        pose = np.eye(4)
        pose[0, 3] = 0        # x = 0
        pose[1, 3] = 10 - i   # y decreasing
        poses.append(pose)

    print(f"Created {len(poses)} poses in a square loop")

    # Add nodes with drift
    for i, pose in enumerate(poses):
        # Add some drift
        noisy_pose = pose.copy()
        noisy_pose[:3, 3] += np.random.randn(3) * 0.1  # Add noise

        graph.add_node(i, noisy_pose, fixed=(i == 0))

    # Add odometry edges
    for i in range(len(poses) - 1):
        relative = np.linalg.inv(poses[i]) @ poses[i + 1]
        # Add noise to odometry
        relative[:3, 3] += np.random.randn(3) * 0.05
        graph.add_odometry_edge(i, i + 1, relative)

    print(f"Added {len(graph.odometry_edges)} odometry edges")

    # Add loop closure (last pose should be close to first)
    loop_measurement = np.eye(4)  # Should be identity (same position)
    graph.add_loop_closure(0, len(poses) - 1, loop_measurement, confidence=0.8)

    print(f"Added loop closure: 0 -> {len(poses) - 1}")

    # Get trajectory before optimization
    traj_before = graph.get_optimized_trajectory()
    print(f"\nBefore optimization:")
    print(f"  Start: {traj_before[0]}")
    print(f"  End: {traj_before[-1]}")
    print(f"  Gap: {np.linalg.norm(traj_before[-1] - traj_before[0]):.3f}m")

    # Optimize
    print("\nOptimizing...")
    final_error = graph.optimize()

    # Get trajectory after optimization
    traj_after = graph.get_optimized_trajectory()
    print(f"\nAfter optimization:")
    print(f"  Start: {traj_after[0]}")
    print(f"  End: {traj_after[-1]}")
    print(f"  Gap: {np.linalg.norm(traj_after[-1] - traj_after[0]):.3f}m")

    print(f"\nStats: {graph.get_stats()}")