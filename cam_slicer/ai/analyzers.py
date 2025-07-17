"""Analysis helpers for toolpath optimization and comparison."""

from typing import List, Tuple, Dict, Optional

from cam_slicer.tool_material_db import ToolMaterialDB
from cam_slicer.plugin_manager import execute_plugin
from cam_slicer.robotics.safety import _angle_between


def feedrate_advisor(
    toolpath: List[Tuple[float, float, float]],
    *,
    tool: Optional[str] = None,
    material: Optional[str] = None,
    db: Optional[ToolMaterialDB] = None,
) -> Dict[str, float]:
    """Suggest feedrate using tool/material data and path length.

    When a tool/material pair is known, the database feedrate is used as the
    baseline. It is then scaled by the average move length to adapt to the
    current job. If no data is available a generic 1000 mm/min is returned.
    """

    if db is None:
        db = ToolMaterialDB()
        try:
            db.load("tool_materials.json")
        except Exception:
            pass

    base_feed = 1000.0
    rpm = 10000
    if tool and material:
        params = db.get_params(tool, material)
        if params:
            base_feed = params.feedrate
            rpm = params.rpm

    if len(toolpath) < 2:
        return {"recommended_feedrate": base_feed, "rpm": rpm}

    total_len = 0.0
    for a, b in zip(toolpath[:-1], toolpath[1:]):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        total_len += (dx * dx + dy * dy + dz * dz) ** 0.5

    avg_len = total_len / (len(toolpath) - 1)
    scale = max(0.5, min(1.5, avg_len))
    feed = base_feed * scale
    return {"recommended_feedrate": round(feed, 2), "rpm": rpm}


def trajectory_cleaner(
    toolpath: List[Tuple[float, float, float]],
    *,
    distance_threshold: float = 0.05,
    angle_threshold: float = 170.0,
) -> Dict[str, object]:
    """Remove duplicates, micro moves and zig-zags from the path."""

    if not toolpath:
        return {"points": []}

    removed = 0
    cleaned = [toolpath[0]]

    for pt in toolpath[1:]:
        last = cleaned[-1]
        dx, dy, dz = pt[0] - last[0], pt[1] - last[1], pt[2] - last[2]
        dist = float((dx * dx + dy * dy + dz * dz) ** 0.5)
        if dist <= distance_threshold:
            removed += 1
            continue
        if len(cleaned) >= 2:
            prev = cleaned[-2]
            vec1 = (last[0] - prev[0], last[1] - prev[1], last[2] - prev[2])
            vec2 = (pt[0] - last[0], pt[1] - last[1], pt[2] - last[2])
            ang = _angle_between(vec1, vec2)
            if ang > angle_threshold and dist < distance_threshold * 5:
                cleaned.pop()
                removed += 2
                if pt != cleaned[-1]:
                    cleaned.append(pt)
                continue
        if pt != last:
            cleaned.append(pt)

    return {"points": cleaned, "removed": removed}


def surface_comparator(toolpath: List[Tuple[float, float, float]]) -> Dict[str, float]:
    """Compute simple surface statistics."""
    if not toolpath:
        return {"min_z": 0.0, "max_z": 0.0}
    zs = [p[2] for p in toolpath]
    return {"min_z": min(zs), "max_z": max(zs)}


def plugin_optimizer(toolpath: List[Tuple[float, float, float]], name: str) -> Dict[str, object]:
    """Run a plugin to optimize the toolpath."""
    optimized = execute_plugin(name, toolpath)
    return {"points": optimized}


def ml_speed_optimizer(toolpath: List[Tuple[float, float, float]]) -> Dict[str, object]:
    """Smooth toolpath points using a simple moving average."""
    if not toolpath:
        return {"points": []}
    smoothed = [toolpath[0]]
    for i in range(1, len(toolpath) - 1):
        prev_pt = toolpath[i - 1]
        cur_pt = toolpath[i]
        next_pt = toolpath[i + 1]
        avg = (
            (prev_pt[0] + cur_pt[0] + next_pt[0]) / 3.0,
            (prev_pt[1] + cur_pt[1] + next_pt[1]) / 3.0,
            (prev_pt[2] + cur_pt[2] + next_pt[2]) / 3.0,
        )
        smoothed.append(avg)
    smoothed.append(toolpath[-1])
    return {"points": smoothed}


__all__ = [
    "feedrate_advisor",
    "trajectory_cleaner",
    "surface_comparator",
    "plugin_optimizer",
    "ml_speed_optimizer",
]
