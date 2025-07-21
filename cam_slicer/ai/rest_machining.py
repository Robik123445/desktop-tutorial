import logging
from typing import List, Tuple

try:
    from shapely.geometry import Polygon, LineString, MultiPolygon  # type: ignore
    from shapely.ops import unary_union
except Exception as exc:  # pragma: no cover - optional dependency
    logging.getLogger(__name__).warning("Shapely not installed: %s", exc)
    Polygon = None  # type: ignore
    LineString = None  # type: ignore
    MultiPolygon = None  # type: ignore
    unary_union = None  # type: ignore

from cam_slicer.logging_config import setup_logging
from cam_slicer.shapes import generate_adaptive_path

setup_logging()
logger = logging.getLogger(__name__)

Point2D = Tuple[float, float]
Point3D = Tuple[float, float, float]


def analyze_remaining_area(
    boundary: List[Point2D],
    roughing_paths: List[List[Point3D]],
    roughing_radius: float,
    rest_radius: float,
) -> MultiPolygon:
    """Return polygon area that still contains material after roughing.

    Parameters
    ----------
