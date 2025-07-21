import logging
from typing import List, Dict, Tuple

try:
    import trimesh  # type: ignore
    from shapely.geometry import Polygon
except Exception as exc:  # pragma: no cover - optional dependency
    logging.getLogger(__name__).warning("Optional deps missing: %s", exc)
    trimesh = None
    Polygon = None

from cam_slicer.logging_config import setup_logging
from cam_slicer.shapes import generate_adaptive_path, generate_spiral_helix

setup_logging()

Feature = Dict[str, object]


def detect_mesh_features(mesh: 'trimesh.Trimesh') -> List[Feature]:
    """Detect basic features such as pockets, holes and slots.

    Uses a mid-plane cross section. Holes are derived from interior rings and
    classified by aspect ratio.
    """
    if trimesh is None or Polygon is None:
        raise ImportError("trimesh and shapely are required for feature detection")

    z_levels = sorted(set(round(v[2], 4) for v in mesh.vertices))
    if len(z_levels) < 2:
        return []
    mid_z = (z_levels[0] + z_levels[-1]) / 2
    section = mesh.section(plane_origin=[0, 0, mid_z], plane_normal=[0, 0, 1])
