import logging
from typing import List, Tuple

try:
    from shapely.geometry import Polygon, LineString, MultiPolygon  # type: ignore
    from shapely.ops import unary_union
except Exception as exc:  # pragma: no cover - optional dependency
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
    boundary : list of tuple
        Outer boundary of the part in XY.
    roughing_paths : list of list
        Paths executed with a larger tool.
    roughing_radius : float
        Radius of the roughing tool.
    rest_radius : float
        Radius of the finishing tool.
    """
    if Polygon is None:
        raise ImportError("shapely is required for rest machining")
    poly = Polygon(boundary)
    if rest_radius >= roughing_radius:
        return MultiPolygon([])
    rough_removed = []
    for seg in roughing_paths:
        ls = LineString([(p[0], p[1]) for p in seg])
        rough_removed.append(ls.buffer(roughing_radius))
    removed = unary_union(rough_removed) if rough_removed else Polygon()
    small_area = poly.buffer(-rest_radius)
    remaining = small_area.difference(removed)
    logger.info("Remaining area has %.3f sq units", remaining.area)
    if isinstance(remaining, Polygon):
        return MultiPolygon([remaining])
    return remaining


def generate_rest_machining_paths(
    boundary: List[Point2D],
    roughing_paths: List[List[Point3D]],
    roughing_radius: float,
    rest_radius: float,
    *,
    depth: float,
    stepdown: float = 0.5,
    stepover: float = 0.3,
    mode: str = "spiral",
) -> List[List[Point3D]]:
    """Generate cleanup toolpaths for leftover material.

    The function computes areas unreachable by the roughing tool and
    generates adaptive passes only within these regions.
    """
    remaining = analyze_remaining_area(boundary, roughing_paths, roughing_radius, rest_radius)
    toolpaths: List[List[Point3D]] = []
    for geom in remaining.geoms:
        if geom.is_empty:
            continue
        coords = [(float(x), float(y)) for x, y in geom.exterior.coords]
        paths = generate_adaptive_path(
            coords,
            depth=depth,
            stepdown=stepdown,
            stepover=stepover,
            mode=mode,
        )
        toolpaths.extend(paths)
        logger.info("Generated %d rest passes for region", len(paths))
    return toolpaths
