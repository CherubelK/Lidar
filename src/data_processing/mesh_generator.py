"""
3D Mesh Generation from Point Clouds
Converts processed point cloud data into 3D meshes for visualization and export.
"""

import numpy as np
from scipy.spatial import Delaunay
from sklearn.neighbors import NearestNeighbors
import logging
from pathlib import Path
from typing import Optional, Tuple, Dict
import json

logger = logging.getLogger(__name__)


class MeshGenerator:
    """
    Generate 3D meshes from point cloud data using various algorithms.
    """

    def __init__(self):
        """Initialize mesh generator."""
        pass

    @staticmethod
    def poisson_surface_reconstruction_fallback(points: np.ndarray,
                                                 normals: np.ndarray,
                                                 depth: int = 8) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simplified surface reconstruction using Delaunay triangulation.
        This is a fallback when Open3D's Poisson reconstruction is not available.

        Args:
            points: Nx3 point coordinates
            normals: Nx3 normal vectors
            depth: Octree depth (not used in this fallback, kept for API compatibility)

        Returns:
            Tuple of (vertices, faces) where faces are triangles
        """
        logger.info("Using Delaunay triangulation for surface reconstruction")

        # Project points to 2D for triangulation (use XY plane)
        points_2d = points[:, :2]

        try:
            # Perform Delaunay triangulation
            tri = Delaunay(points_2d)

            vertices = points
            faces = tri.simplices

            # Filter faces based on edge length to remove long triangles
            max_edge_length = 0.5  # meters
            valid_faces = []

            for face in faces:
                v0, v1, v2 = vertices[face]
                edge1 = np.linalg.norm(v1 - v0)
                edge2 = np.linalg.norm(v2 - v1)
                edge3 = np.linalg.norm(v0 - v2)

                if max(edge1, edge2, edge3) < max_edge_length:
                    valid_faces.append(face)

            faces = np.array(valid_faces)

            logger.info(f"Generated mesh with {len(vertices)} vertices and {len(faces)} faces")
            return vertices, faces

        except Exception as e:
            logger.error(f"Delaunay triangulation failed: {e}")
            return points, np.array([])

    @staticmethod
    def ball_pivoting(points: np.ndarray,
                     normals: np.ndarray,
                     radii: list = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Ball-pivoting algorithm for mesh reconstruction.
        Simplified implementation using alpha shapes concept.

        Args:
            points: Nx3 point coordinates
            normals: Nx3 normal vectors
            radii: List of ball radii to try

        Returns:
            Tuple of (vertices, faces)
        """
        logger.info("Using simplified ball-pivoting mesh reconstruction")

        if radii is None:
            radii = [0.05, 0.1, 0.2]

        # Use nearest neighbors to build mesh
        nbrs = NearestNeighbors(n_neighbors=20, algorithm='ball_tree').fit(points)
        distances, indices = nbrs.kneighbors(points)

        # Build faces from local neighborhoods
        faces = []
        for i, neighbors in enumerate(indices):
            # Create triangles from nearest neighbors
            for j in range(len(neighbors) - 2):
                face = [i, neighbors[j], neighbors[j + 1]]
                faces.append(face)

        faces = np.array(faces)

        # Remove duplicate faces
        faces = np.unique(np.sort(faces, axis=1), axis=0)

        logger.info(f"Generated mesh with {len(points)} vertices and {len(faces)} faces")
        return points, faces

    def create_trail_mesh(self,
                         points: np.ndarray,
                         normals: Optional[np.ndarray] = None,
                         method: str = 'delaunay') -> Dict:
        """
        Create a complete mesh from trail point cloud.

        Args:
            points: Nx3 point coordinates
            normals: Nx3 normal vectors (optional)
            method: 'delaunay' or 'ball_pivoting'

        Returns:
            Dictionary containing mesh data
        """
        logger.info(f"Creating trail mesh using {method} method")

        if normals is None:
            logger.info("No normals provided, estimating...")
            from .point_cloud_processor_numpy import PointCloudProcessorNumPy
            processor = PointCloudProcessorNumPy()
            points, normals = processor.estimate_normals(points)

        if method == 'delaunay':
            vertices, faces = self.poisson_surface_reconstruction_fallback(points, normals)
        elif method == 'ball_pivoting':
            vertices, faces = self.ball_pivoting(points, normals)
        else:
            raise ValueError(f"Unknown method: {method}")

        mesh_data = {
            'vertices': vertices,
            'faces': faces,
            'normals': normals,
            'num_vertices': len(vertices),
            'num_faces': len(faces)
        }

        logger.info(f"Mesh created: {mesh_data['num_vertices']} vertices, {mesh_data['num_faces']} faces")
        return mesh_data

    @staticmethod
    def export_to_obj(mesh_data: Dict, output_path: str) -> None:
        """
        Export mesh to OBJ format (widely compatible).

        Args:
            mesh_data: Mesh dictionary from create_trail_mesh()
            output_path: Path to save OBJ file
        """
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        vertices = mesh_data['vertices']
        faces = mesh_data['faces']
        normals = mesh_data.get('normals', None)

        with open(output_path, 'w') as f:
            f.write("# Trail mesh generated by Unitree L2 LiDAR Trail Mapper\n")
            f.write(f"# Vertices: {len(vertices)}\n")
            f.write(f"# Faces: {len(faces)}\n\n")

            # Write vertices
            for vertex in vertices:
                f.write(f"v {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")

            # Write normals if available
            if normals is not None:
                for normal in normals:
                    f.write(f"vn {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}\n")

            # Write faces (OBJ uses 1-based indexing)
            for face in faces:
                if normals is not None:
                    f.write(f"f {face[0]+1}//{face[0]+1} {face[1]+1}//{face[1]+1} {face[2]+1}//{face[2]+1}\n")
                else:
                    f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")

        logger.info(f"Exported mesh to OBJ: {output_path}")

    @staticmethod
    def export_to_ply(mesh_data: Dict, output_path: str) -> None:
        """
        Export mesh to PLY format (good for 3D software).

        Args:
            mesh_data: Mesh dictionary from create_trail_mesh()
            output_path: Path to save PLY file
        """
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        vertices = mesh_data['vertices']
        faces = mesh_data['faces']
        normals = mesh_data.get('normals', None)

        with open(output_path, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(vertices)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")

            if normals is not None:
                f.write("property float nx\n")
                f.write("property float ny\n")
                f.write("property float nz\n")

            f.write(f"element face {len(faces)}\n")
            f.write("property list uchar int vertex_indices\n")
            f.write("end_header\n")

            # Write vertex data
            for i, vertex in enumerate(vertices):
                if normals is not None:
                    f.write(f"{vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f} ")
                    f.write(f"{normals[i][0]:.6f} {normals[i][1]:.6f} {normals[i][2]:.6f}\n")
                else:
                    f.write(f"{vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")

            # Write face data
            for face in faces:
                f.write(f"3 {face[0]} {face[1]} {face[2]}\n")

        logger.info(f"Exported mesh to PLY: {output_path}")

    @staticmethod
    def export_to_json(mesh_data: Dict, output_path: str) -> None:
        """
        Export mesh to JSON format (for web visualization with Three.js).

        Args:
            mesh_data: Mesh dictionary from create_trail_mesh()
            output_path: Path to save JSON file
        """
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Simple format for web viewer
        export_data = {
            "vertices": mesh_data['vertices'].flatten().tolist(),
            "faces": mesh_data['faces'].flatten().tolist()
        }

        with open(output_path, 'w') as f:
            json.dump(export_data, f)

        logger.info(f"Exported mesh to JSON: {output_path}")

    def export_all_formats(self, mesh_data: Dict, base_path: str) -> Dict[str, str]:
        """
        Export mesh in all supported formats.

        Args:
            mesh_data: Mesh dictionary
            base_path: Base path without extension

        Returns:
            Dictionary mapping format to file path
        """
        base_path = Path(base_path)
        base_name = base_path.stem
        output_dir = base_path.parent

        exports = {}

        # OBJ format
        obj_path = output_dir / f"{base_name}.obj"
        self.export_to_obj(mesh_data, str(obj_path))
        exports['obj'] = str(obj_path)

        # PLY format
        ply_path = output_dir / f"{base_name}.ply"
        self.export_to_ply(mesh_data, str(ply_path))
        exports['ply'] = str(ply_path)

        # JSON format (for web)
        json_path = output_dir / f"{base_name}.json"
        self.export_to_json(mesh_data, str(json_path))
        exports['json'] = str(json_path)

        logger.info(f"Exported mesh in {len(exports)} formats")
        return exports