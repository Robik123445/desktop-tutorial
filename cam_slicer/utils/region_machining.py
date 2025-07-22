"""Region-constrained toolpath clipping utilities."""

from typing import List, Tuple, Iterable
import logging

try:
    from shapely.geometry import Polygon, Point  # type: ignore
    from shapely.ops import unary_union
except Exception as exc:  # pragma: no cover - optional
    logging.getLogger(__name__).warning("Shapely not installed: %s", exc)
    Polygon = None  # type: ignore
    Point = None  # type: ignore
    unary_union = None  # type: ignore

# Use the central logging setup for consistency across modules
from cam_slicer.logging_config import setup_logging
setup_logging()
logger = logging.getLogger(__name__)

Point2D = Tuple[float, float]
Point3D = Tuple[float, float, float]


def clip_toolpath_to_regions(toolpath: List[Point3D], regions: Iterable[List[Point2D]]) -> List[Point3D]:
    """Return toolpath limited to union of user regions.

    Parameters
    ----------
    toolpath : list of tuple
        Original toolpath ``[(x, y, z), ...]``.
    regions : iterable of list
        Polygons describing machining regions ``[(x, y), ...]``.

    Returns
    -------
    list of tuple
        Toolpath with points outside the regions removed.
    """
    if Polygon is None:
        raise ImportError("shapely is required for region clipping")
    polys = [Polygon(r) for r in regions if len(r) >= 3]
