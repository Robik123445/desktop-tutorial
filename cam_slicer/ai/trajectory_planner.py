"""Trajectory planning helpers like arc smoothing and lookahead."""

import logging
from typing import Dict, List, Tuple, Union, Optional

from cam_slicer.logging_config import setup_logging

setup_logging()


def _angle_between(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """Return angle in degrees between two 2D vectors."""
    dot = a[0] * b[0] + a[1] * b[1]
    mag1 = (a[0] ** 2 + a[1] ** 2) ** 0.5
    mag2 = (b[0] ** 2 + b[1] ** 2) ** 0.5
    if mag1 == 0 or mag2 == 0:
        return 0.0
    cos_theta = max(-1.0, min(1.0, dot / (mag1 * mag2)))
    from math import acos, degrees

    return degrees(acos(cos_theta))


def _circle_from_points(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> Optional[Tuple[Tuple[float, float], float, str]]:
    """Return circle center, radius and orientation from three points."""
    ax, ay = a
    bx, by = b
    cx, cy = c
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-6:
        return None
    ux = ((ax ** 2 + ay ** 2) * (by - cy) + (bx ** 2 + by ** 2) * (cy - ay) + (cx ** 2 + cy ** 2) * (ay - by)) / d
    uy = ((ax ** 2 + ay ** 2) * (cx - bx) + (bx ** 2 + by ** 2) * (ax - cx) + (cx ** 2 + cy ** 2) * (bx - ax)) / d
    radius = ((ax - ux) ** 2 + (ay - uy) ** 2) ** 0.5
    cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    direction = "CCW" if cross > 0 else "CW"
    return (ux, uy), radius, direction


def smooth_toolpath_corners(
    toolpath: List[Tuple[float, float, float]],
    angle_threshold: float = 30.0,
) -> List[Union[Tuple[float, float, float], Tuple[str, Dict[str, object]]]]:
    """Insert arc moves when corner angle exceeds ``angle_threshold``.

    Parameters
    ----------
    toolpath : list of tuple
        ``[(x, y, z), ...]`` coordinates.
    angle_threshold : float
        Minimum corner angle that triggers arc insertion.

    Returns
    -------
    list
        Sequence of points and arc instructions.

    Example
    -------
    >>> path = [(0,0,0), (1,0,0), (1,1,0)]
    >>> smooth_toolpath_corners(path, 45)
    [(0,0,0), ('arc', {...}), (1,1,0)]
    """
    if len(toolpath) < 3:
        return toolpath[:]

    result: List[Union[Tuple[float, float, float], Tuple[str, Dict[str, object]]]] = [toolpath[0]]
    i = 1
    while i < len(toolpath) - 1:
        prev = toolpath[i - 1]
        cur = toolpath[i]
        nxt = toolpath[i + 1]
        v1 = (cur[0] - prev[0], cur[1] - prev[1])
        v2 = (nxt[0] - cur[0], nxt[1] - cur[1])
        angle = _angle_between(v1, v2)
        if angle > angle_threshold:
            circle = _circle_from_points(prev[:2], cur[:2], nxt[:2])
            if circle:
                center_xy, radius, direction = circle
                center = (center_xy[0], center_xy[1], cur[2])
                arc = {
                    "start": prev,
                    "end": nxt,
                    "center": center,
                    "radius": radius,
                    "direction": direction,
                }
                logging.info(
                    "Corner at index %d smoothed with %s arc of r=%.3f",
                    i,
                    direction,
                    radius,
                )
                result.append(("arc", arc))
                result.append(nxt)
                i += 2
                continue
        result.append(cur)
        i += 1

    if i == len(toolpath) - 1:
        result.append(toolpath[-1])

    return result
