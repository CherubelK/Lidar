"""
Material Classification from LiDAR Reflectivity
The Unitree L2 captures a reflectivity intensity (0-255) per point, but
nothing downstream of the UDP parser currently reads it -- it's saved to
disk in map_merger.py and never analyzed. This module turns that unused
channel into a per-point material/surface signal.

Raw intensity is confounded by distance and angle of incidence (return
power falls off roughly with 1/r^2 for diffuse reflectors, and with the
cosine of the incidence angle for Lambertian surfaces), so it's normalized
before classification. The band thresholds below are reasonable starting
points, not a trained/calibrated model -- they should be tuned against
real scans of known materials before being trusted for a specific
deployment (e.g. distinguishing cardboard from metal racking).
"""
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Optional
import logging

logger = logging.getLogger(__name__)


@dataclass
class MaterialBand:
    """A normalized-intensity range mapped to a candidate surface type."""
    name: str
    min_intensity: float
    max_intensity: float
    description: str


# Default bands over normalized intensity in [0, 1]. Tune against labeled
# scans of your actual target surfaces before relying on these.
DEFAULT_BANDS: List[MaterialBand] = [
    MaterialBand("low_reflectivity", 0.0, 0.15,
                 "Dark/absorptive surfaces: black plastic, dark fabric, voids"),
    MaterialBand("medium_reflectivity", 0.15, 0.45,
                 "Typical diffuse surfaces: cardboard, drywall, wood, most packaging"),
    MaterialBand("high_reflectivity", 0.45, 0.75,
                 "Light-colored or smooth surfaces: painted metal, plastic bins"),
    MaterialBand("very_high_reflectivity", 0.75, 1.01,
                 "Retroreflective/specular: safety tape, bare metal, glass-adjacent returns"),
]


class MaterialClassifier:
    """
    Classifies LiDAR points into reflectivity bands after correcting raw
    intensity for distance falloff, optionally producing spatial clusters
    of same-band points (e.g. a contiguous patch of likely safety tape).
    """

    def __init__(self, bands: Optional[List[MaterialBand]] = None):
        self.bands = bands or DEFAULT_BANDS

    def normalize_intensity(self, points: np.ndarray, intensities: np.ndarray,
                             sensor_origin: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Correct raw 0-1 intensity for distance falloff.

        Args:
            points: Nx3 point coordinates
            intensities: Nx1 (or N,) raw intensity in [0, 1]
                (as produced by UnitreeL2UDP._parse_3d_point_data)
            sensor_origin: 3-vector sensor position; defaults to the
                origin, which is correct for raw sensor-frame points but
                not for points already transformed into a SLAM world frame.

        Returns:
            Nx1 distance-corrected intensity, clipped to [0, 1].
        """
        intensities = np.asarray(intensities).reshape(-1)
        origin = np.zeros(3) if sensor_origin is None else np.asarray(sensor_origin)

        distances = np.linalg.norm(points - origin, axis=1)
        distances = np.clip(distances, 0.1, None)  # avoid blow-up near zero range

        # Diffuse return power falls off ~1/r^2; undo that to recover a
        # distance-independent reflectivity estimate.
        corrected = intensities * (distances ** 2)

        # Re-normalize into [0, 1] using the observed range in this scan.
        max_val = corrected.max() if corrected.max() > 0 else 1.0
        corrected = np.clip(corrected / max_val, 0.0, 1.0)

        return corrected.reshape(-1, 1)

    def classify(self, points: np.ndarray, intensities: np.ndarray,
                 sensor_origin: Optional[np.ndarray] = None,
                 already_normalized: bool = False) -> Dict[str, np.ndarray]:
        """
        Assign each point to a material band.

        Args:
            points: Nx3 point coordinates
            intensities: Nx1 raw (or pre-normalized) intensity
            sensor_origin: sensor position for distance correction
            already_normalized: skip distance correction if intensities
                were already normalized upstream

        Returns:
            Dict mapping band name -> Nx3 points assigned to that band.
        """
        normalized = (
            np.asarray(intensities).reshape(-1)
            if already_normalized
            else self.normalize_intensity(points, intensities, sensor_origin).reshape(-1)
        )

        result: Dict[str, np.ndarray] = {}
        for band in self.bands:
            mask = (normalized >= band.min_intensity) & (normalized < band.max_intensity)
            result[band.name] = points[mask]
            logger.debug(f"Band '{band.name}': {mask.sum()} points")

        return result

    def flag_band(self, points: np.ndarray, intensities: np.ndarray, band_name: str,
                   sensor_origin: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Convenience helper to pull out points in a single named band, e.g.
        flag_band(points, intensities, "very_high_reflectivity") to locate
        likely safety tape/retroreflective markers in a warehouse scan.
        """
        classified = self.classify(points, intensities, sensor_origin)
        if band_name not in classified:
            raise ValueError(f"Unknown band '{band_name}'. Available: {list(classified.keys())}")
        return classified[band_name]
