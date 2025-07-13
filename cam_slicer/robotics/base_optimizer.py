from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Iterable, List, Sequence, Tuple

try:
    from cam_slicer.utils.geofence import GeoFence
except Exception:  # pragma: no cover - optional import
    GeoFence = None  # type: ignore

from cam_slicer.logging_config import setup_logging
from .interface import ArmKinematicProfile, format_extended_g1

setup_logging()
logger = logging.getLogger(__name__)

@dataclass
class BaseZone:
    """Zone with base position and assigned toolpath segment."""
    base: Tuple[float, float]
    toolpath: List[Sequence[float]]


@dataclass
class ZonePlan(BaseZone):
    """Planned zone with move commands and potential warnings."""

    move_cmds: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


def _candidate_positions(x_range: Tuple[float, float], y_range: Tuple[float, float], step: float) -> Iterable[Tuple[float, float]]:
    x0, x1 = x_range
    y0, y1 = y_range
    x = x0
    while x <= x1:
        y = y0
        while y <= y1:
            yield (x, y)
            y += step
        x += step


def _cost_for_base(
    toolpath: List[Sequence[float]],
    profile: ArmKinematicProfile,
    base: Tuple[float, float],
    obstacles: "GeoFence" | None = None,
) -> float | None:
    cost = 0.0
    bx, by = base
    for pt in toolpath:
        shifted = (pt[0] - bx, pt[1] - by, pt[2] if len(pt) > 2 else 0.0)
        try:
            joints = profile.workspace_to_joints(shifted)
        except Exception:
            return None
        if obstacles and profile.check_collision(joints, obstacles):
            return None
        cost += sum(abs(a) for a in joints)
    return cost


def optimize_base_position(
    toolpath: List[Sequence[float]],
    profile: ArmKinematicProfile,
    base_x_range: Tuple[float, float],
    base_y_range: Tuple[float, float],
    *,
    step: float = 50.0,
    obstacles: "GeoFence" | None = None,
) -> Tuple[float, float] | None:
    """Return best base position so all points are reachable."""
    best: Tuple[float, float] | None = None
    best_cost = float("inf")
    for cand in _candidate_positions(base_x_range, base_y_range, step):
        if obstacles and obstacles.is_inside(*cand):
            continue
        cost = _cost_for_base(toolpath, profile, cand, obstacles)
        if cost is not None and cost < best_cost:
            best = cand
            best_cost = cost
    if best is not None:
        logger.info("Base optimized to %s with cost %.1f", best, best_cost)
    else:
        logger.warning("No valid base position found")
    return best


def optimize_base_position_ai(
    toolpath: List[Sequence[float]],
    profile: ArmKinematicProfile,
    base_x_range: Tuple[float, float],
    base_y_range: Tuple[float, float],
    *,
    obstacles: "GeoFence" | None = None,
    step: float = 50.0,
    obstacle_margin: float = 50.0,
) -> Tuple[float, float] | None:
    """Select best base using heuristics with obstacle avoidance."""

    if GeoFence is None and obstacles is not None:  # pragma: no cover - missing deps
        obstacles = None

    centroid_x = sum(p[0] for p in toolpath) / len(toolpath)
    centroid_y = sum(p[1] for p in toolpath) / len(toolpath)
    best: Tuple[float, float] | None = None
    best_cost = float("inf")
    for cand in _candidate_positions(base_x_range, base_y_range, step):
        if obstacles and obstacles.is_inside(*cand):
            continue
        cost = _cost_for_base(toolpath, profile, cand)
        if cost is None:
            continue
        dist_centroid = ((cand[0] - centroid_x) ** 2 + (cand[1] - centroid_y) ** 2) ** 0.5
        obstacle_penalty = 0.0
        if obstacles:
            dists = []
            for x1, y1, x2, y2 in obstacles.forbidden_zones:
                cx = max(min(cand[0], max(x1, x2)), min(x1, x2))
                cy = max(min(cand[1], max(y1, y2)), min(y1, y2))
                dists.append(((cand[0] - cx) ** 2 + (cand[1] - cy) ** 2) ** 0.5)
            if dists:
                near = min(dists)
                if near < obstacle_margin:
                    obstacle_penalty = (obstacle_margin - near) * 10
        total_cost = cost + dist_centroid + obstacle_penalty
        if total_cost < best_cost:
            best = cand
            best_cost = total_cost
    if best:
        logger.info("AI optimized base to %s with cost %.1f", best, best_cost)
    else:
        logger.warning("AI could not find valid base position")
    return best


def plan_base_zones(
    toolpath: List[Sequence[float]],
    profile: ArmKinematicProfile,
    base_x_range: Tuple[float, float],
    base_y_range: Tuple[float, float],
    *,
    step: float = 50.0,
    obstacles: "GeoFence" | None = None,
) -> List[BaseZone]:
    """Split toolpath into zones with optimized base positions."""
    remaining = list(toolpath)
    zones: List[BaseZone] = []
    while remaining:
        best_zone: BaseZone | None = None
        for cand in _candidate_positions(base_x_range, base_y_range, step):
            if obstacles and obstacles.is_inside(*cand):
                continue
            segment: List[Sequence[float]] = []
            for pt in remaining:
                shifted = (pt[0] - cand[0], pt[1] - cand[1], pt[2] if len(pt) > 2 else 0.0)
                try:
                    profile.workspace_to_joints(shifted)
                except Exception:
                    break
                segment.append(pt)
            if not segment:
                continue
            cost = _cost_for_base(segment, profile, cand, obstacles) or float("inf")
            if best_zone is None:
                best_zone = BaseZone(base=cand, toolpath=segment)
                continue
            best_cost = _cost_for_base(best_zone.toolpath, profile, best_zone.base, obstacles) or float("inf")
            if (len(segment) > len(best_zone.toolpath)) or (
                len(segment) == len(best_zone.toolpath) and cost < best_cost
            ):
                best_zone = BaseZone(base=cand, toolpath=segment)
        if best_zone is None:
            logger.error("Cannot place remaining toolpath within workspace")
            break
        zones.append(best_zone)
        remaining = remaining[len(best_zone.toolpath):]
    logger.info("Planned %d zones", len(zones))
    return zones


def _zone_distance(a: BaseZone, b: BaseZone) -> float:
    ax, ay = a.base
    bx, by = b.base
    return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5


def _simulate_base_move(
    start: Tuple[float, float],
    end: Tuple[float, float],
    *,
    step: float = 10.0,
    obstacles: "GeoFence" | None = None,
) -> bool:
    """Return ``True`` if linear base move avoids obstacles."""

    if obstacles is None:
        return True

    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    dist = (dx**2 + dy**2) ** 0.5
    if dist == 0:
        return not obstacles.is_inside(sx, sy)

    steps = max(1, int(dist / step))
    for i in range(steps + 1):
        t = i / steps
        x = sx + dx * t
        y = sy + dy * t
        if obstacles.is_inside(x, y):
            logger.warning("Base path intersects forbidden zone at %.1f, %.1f", x, y)
            return False
    return True


def plan_workspace_zones(
    toolpath: List[Sequence[float]],
    profile: ArmKinematicProfile,
    base_x_range: Tuple[float, float],
    base_y_range: Tuple[float, float],
    *,
    step: float = 50.0,
    safe_z: float = 50.0,
    obstacles: "GeoFence" | None = None,
    ai_optimize: bool = False,
) -> List[ZonePlan]:
    """Return zones with base moves and optimized order."""

    raw_zones = plan_base_zones(
        toolpath,
        profile,
        base_x_range,
        base_y_range,
        step=step,
        obstacles=obstacles,
    )
    if not raw_zones:
        return []

    remaining = raw_zones.copy()
    ordered: List[BaseZone] = []
    current = remaining.pop(0)
    ordered.append(current)
    while remaining:
        next_zone = min(remaining, key=lambda z: _zone_distance(current, z))
        remaining.remove(next_zone)
        ordered.append(next_zone)
        current = next_zone

    if ai_optimize and len(ordered) > 2:
        coords = [z.base for z in ordered]

        def dist(i: int, j: int) -> float:
            ax, ay = coords[i]
            bx, by = coords[j]
            return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5

        order = list(range(len(coords)))
        improved = True
        while improved:
            improved = False
            for i in range(1, len(order) - 1):
                for j in range(i + 1, len(order)):
                    a, b = order[i - 1], order[i]
                    c, d = order[j], order[j + 1] if j + 1 < len(order) else order[0]
                    current = dist(a, b) + dist(c, d)
                    new = dist(a, c) + dist(b, d)
                    if new < current:
                        order[i : j + 1] = reversed(order[i : j + 1])
                        improved = True
        ordered = [ordered[i] for i in order]

    plans: List[ZonePlan] = []
    prev_end = None
    prev_base = None
    for idx, zone in enumerate(ordered):
        moves: List[str] = []
        warnings: List[str] = []
        if idx == 0:
            moves.append(f"MOVE_BASE X{zone.base[0]:.1f} Y{zone.base[1]:.1f}")
        else:
            if prev_end is not None:
                try:
                    profile.workspace_to_joints((prev_end[0] - prev_base[0], prev_end[1] - prev_base[1], safe_z))
                except Exception as exc:
                    warnings.append(f"Unreachable retract pose: {exc}")
                moves.append(format_extended_g1(z=safe_z))
                if prev_base is not None and not _simulate_base_move(prev_base, zone.base, step=step, obstacles=obstacles):
                    warnings.append("Potential collision during base move")
                moves.append(f"MOVE_BASE X{zone.base[0]:.1f} Y{zone.base[1]:.1f}")
                sx, sy = zone.toolpath[0][0], zone.toolpath[0][1]
                if not _simulate_base_move((zone.base[0], zone.base[1]), (sx, sy), step=step, obstacles=obstacles):
                    warnings.append("Potential collision approaching start")
                try:
                    profile.workspace_to_joints((sx - zone.base[0], sy - zone.base[1], safe_z))
                except Exception as exc:
                    warnings.append(f"Unreachable start pose: {exc}")
                moves.append(format_extended_g1(x=sx, y=sy, z=safe_z))
        if idx < len(ordered) - 1:
            moves.append("M5")
        prev_end = zone.toolpath[-1]
        prev_base = zone.base
        plans.append(
            ZonePlan(base=zone.base, toolpath=zone.toolpath, move_cmds=moves, warnings=warnings)
        )

    logger.info("Optimized into %d zones", len(plans))
    return plans
