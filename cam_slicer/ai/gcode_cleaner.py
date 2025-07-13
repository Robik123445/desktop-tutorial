"""G-code cleanup utilities for air moves and ordering."""

import logging
from typing import List, Tuple

from .trajectory_planner import _angle_between

from cam_slicer.logging_config import setup_logging
from cam_slicer.performance import profiled

setup_logging()


def _dist(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """2D distance helper."""
    from math import hypot

    return hypot(p1[0] - p2[0], p1[1] - p2[1])


def optimize_toolpath(
    toolpath: List[Tuple[float, float, float]],
    *,
    angle_threshold: float = 120.0,
    distance_threshold: float = 0.5,
) -> List[List[Tuple[float, float, float]]]:
    """Remove short air moves and split toolpath into segments with arcs.

    Example
    -------
    >>> path = [(0,0,1), (0.05,0,1), (1,0,0)]
    >>> optimize_toolpath(path)
    [[(0,0,1), (1,0,0)]]
    """
    if len(toolpath) < 2:
        return [toolpath]

    points = [toolpath[0]]
    for p in toolpath[1:]:
        prev = points[-1]
        if prev[2] > 0 and p[2] > 0 and _dist(prev[:2], p[:2]) < distance_threshold:
            logging.info("Removing short air move %.2f mm", _dist(prev[:2], p[:2]))
            continue
        points.append(p)

    segments: List[List[Tuple[float, float, float]]] = []
    i = 0
    while i < len(points) - 1:
        if 0 < i < len(points) - 1:
            prev = points[i - 1]
            cur = points[i]
            nxt = points[i + 1]
            v1 = (cur[0] - prev[0], cur[1] - prev[1])
            v2 = (nxt[0] - cur[0], nxt[1] - cur[1])
            angle = _angle_between(v1, v2)
            if angle >= angle_threshold:
                segments.append([prev, cur, nxt])
                i += 1
                continue
        segments.append([points[i], points[i + 1]])
        i += 1

    return segments


from cam_slicer.performance import profiled


@profiled
def optimize_toolpath_sequence(toolpaths: List[List[Tuple[float, float, float]]]) -> List[List[Tuple[float, float, float]]]:
    """Reorder segments to minimize rapid travel time.

    Example
    -------
    >>> paths = [[(0,0,0),(1,0,0)], [(5,0,0),(5,1,0)]]
    >>> optimize_toolpath_sequence(paths)
    [[(0,0,0),(1,0,0)], [(1,0,0),(5,0,0)], [(5,0,0),(5,1,0)]]
    """
    if not toolpaths:
        return []

    remaining = list(toolpaths)
    ordered: List[List[Tuple[float, float, float]]] = [remaining.pop(0)]
    while remaining:
        last_end = ordered[-1][-1][:2]
        nearest_idx = min(range(len(remaining)), key=lambda i: _dist(last_end, remaining[i][0][:2]))
        ordered.append(remaining.pop(nearest_idx))
        logging.info("Selected next path with distance %.2f mm", _dist(last_end, ordered[-1][0][:2]))

    result: List[List[Tuple[float, float, float]]] = [ordered[0]]
    for prev, nxt in zip(ordered[:-1], ordered[1:]):
        rapid = [(prev[-1][0], prev[-1][1], prev[-1][2]), (nxt[0][0], nxt[0][1], prev[-1][2])]
        result.append(rapid)
        result.append(nxt)
    return result


@profiled
def optimize_toolpath_sequence_tsp(toolpaths: List[List[Tuple[float, float, float]]]) -> List[List[Tuple[float, float, float]]]:
    """Improve ordering using a simple 2-opt TSP solver."""
    if not toolpaths:
        return []

    order = list(range(len(toolpaths)))

    def path_end(idx: int) -> Tuple[float, float]:
        return toolpaths[idx][-1][:2]

    def path_start(idx: int) -> Tuple[float, float]:
        return toolpaths[idx][0][:2]

    best = order[:]
    improved = True
    while improved:
        improved = False
        for i in range(1, len(order) - 1):
            for j in range(i + 1, len(order)):
                a, b = best[i - 1], best[i]
                c, d = best[j], best[(j + 1) % len(order)] if j + 1 < len(order) else best[0]
                current = _dist(path_end(a), path_start(b)) + _dist(path_end(c), path_start(d))
                new = _dist(path_end(a), path_start(c)) + _dist(path_end(b), path_start(d))
                if new < current:
                    best[i:j+1] = reversed(best[i:j+1])
                    improved = True
    ordered = [toolpaths[i] for i in best]
    result: List[List[Tuple[float, float, float]]] = [ordered[0]]
    for prev, nxt in zip(ordered[:-1], ordered[1:]):
        rapid = [(prev[-1][0], prev[-1][1], prev[-1][2]), (nxt[0][0], nxt[0][1], prev[-1][2])]
        result.append(rapid)
        result.append(nxt)
    return result
