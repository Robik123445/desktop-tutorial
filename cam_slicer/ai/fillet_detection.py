import logging
import math
from typing import List, Tuple

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)

Point = Tuple[float, float, float]


def _circle_radius(p0: Point, p1: Point, p2: Point) -> float:
    """Return radius of circle through three points or ``inf`` if collinear."""
    ax, ay, _ = p0
    bx, by, _ = p1
    cx, cy, _ = p2
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-6:
        return float("inf")
    ux = (
        (ax**2 + ay**2) * (by - cy)
        + (bx**2 + by**2) * (cy - ay)
        + (cx**2 + cy**2) * (ay - by)
    ) / d
    uy = (
        (ax**2 + ay**2) * (cx - bx)
        + (bx**2 + by**2) * (ax - cx)
        + (cx**2 + cy**2) * (bx - ax)
    ) / d
    return math.hypot(ax - ux, ay - uy)


def detect_fillet_features(
    path: List[Point],
    cutter_radius: float,
    *,
    angle_threshold: float = 135.0,
) -> List[int]:
    """Detect fillet indices in a toolpath.

    Parameters
    ----------
    path : list of tuple
        Toolpath points ``(x, y, z)``.
    cutter_radius : float
        Cutter radius used for finishing.
    angle_threshold : float, optional
        Max inside angle to qualify as a fillet.

    Returns
    -------
    list of int
        Indices of path points that are likely fillet centers.
    """
    indices: List[int] = []
    for i in range(1, len(path) - 1):
        p0, p1, p2 = path[i - 1], path[i], path[i + 1]
        v1 = (p0[0] - p1[0], p0[1] - p1[1])
        v2 = (p2[0] - p1[0], p2[1] - p1[1])
        l1 = math.hypot(*v1)
        l2 = math.hypot(*v2)
        if l1 < 1e-6 or l2 < 1e-6:
            continue
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        angle = math.degrees(math.acos(max(-1.0, min(1.0, dot / (l1 * l2)))))
        if angle > angle_threshold:
            continue
        radius = _circle_radius(p0, p1, p2)
        if radius <= cutter_radius * 1.5:
            indices.append(i)
            logger.debug("Fillet at index %d radius %.3f", i, radius)
    return indices


def generate_fillet_tracing(
    path: List[Point],
    cutter_radius: float,
    *,
    segments: int = 8,
) -> List[List[Point]]:
    """Generate circular finishing paths at detected fillets."""
    arcs: List[List[Point]] = []
    for idx in detect_fillet_features(path, cutter_radius):
        center = path[idx]
        arc: List[Point] = []
        for k in range(segments + 1):
            a = 2 * math.pi * k / segments
            x = center[0] + cutter_radius * math.cos(a)
            y = center[1] + cutter_radius * math.sin(a)
            arc.append((x, y, center[2]))
        arcs.append(arc)
        logger.info("Generated fillet pass around point %s", center)
    return arcs
