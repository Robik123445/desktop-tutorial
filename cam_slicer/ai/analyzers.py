"""Simplified analysis utilities for API use."""

from typing import List, Tuple, Dict


def feedrate_advisor(toolpath: List[Tuple[float, float, float]]) -> Dict[str, float]:
    """Return basic feedrate recommendation based on average segment length."""
    if len(toolpath) < 2:
        return {"recommended_feedrate": 1000.0}
    total_len = 0.0
    for a, b in zip(toolpath[:-1], toolpath[1:]):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        total_len += (dx * dx + dy * dy + dz * dz) ** 0.5
    avg_len = total_len / (len(toolpath) - 1)
    feed = max(200.0, min(1500.0, avg_len * 1000))
    return {"recommended_feedrate": round(feed, 2)}


def trajectory_cleaner(toolpath: List[Tuple[float, float, float]]) -> Dict[str, object]:
    """Remove repeated points from toolpath."""
    if not toolpath:
        return {"points": []}
    cleaned = [toolpath[0]]
    for pt in toolpath[1:]:
        if pt != cleaned[-1]:
            cleaned.append(pt)
    return {"points": cleaned, "removed": len(toolpath) - len(cleaned)}


def surface_comparator(toolpath: List[Tuple[float, float, float]]) -> Dict[str, float]:
    """Compute simple surface statistics."""
    if not toolpath:
        return {"min_z": 0.0, "max_z": 0.0}
    zs = [p[2] for p in toolpath]
    return {"min_z": min(zs), "max_z": max(zs)}
