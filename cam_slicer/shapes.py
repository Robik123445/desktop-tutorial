import logging
import math
from typing import List, Tuple

from cam_slicer.logging_config import setup_logging
from cam_slicer.ai.adaptive_steps import compute_adaptive_steps

setup_logging()


def generate_adaptive_path(
    boundary: List[Tuple[float, float]],
    depth: float,
    stepdown: float,
    stepover: float,
    mode: str = "spiral",
    engagement_angle: float = 90.0,
    adaptive_mode: bool = False,
    material_hardness: float = 1.0,
    geometry_factor: float = 0.5,
    tool_load: float = 0.5,
) -> List[List[Tuple[float, float, float]]]:
    """Generate spiral or zigzag toolpaths with near-constant engagement.

    Parameters
    ----------
    boundary : list of tuple
        Outline of the pocket or part ``[(x, y), ...]``.
    depth : float
        Final Z depth. Negative for milling downwards.
    stepdown : float
        Z step for each layer. Must be positive.
    stepover : float
        Lateral step between passes. Used as radial reduction in spiral mode.
    mode : str, optional
        ``"spiral"`` (default) or ``"zigzag"``.
    engagement_angle : float, optional
        Desired tool engagement angle in degrees for spiral mode. ``stepover``
        is converted into a tangential move so the radial and tangential motion
        keep roughly this angle.
    adaptive_mode : bool, optional
        Experimental mode that slightly adjusts ``stepdown`` and ``stepover``
        per layer to follow surface variations.
    material_hardness : float, optional
        Relative hardness ``>=1``; higher values reduce stepdown/stepover.
    geometry_factor : float, optional
        Complexity ``0-1``; higher means more detail so cuts are smaller.
    tool_load : float, optional
        Current load on the tool ``0-1`` used to further reduce step size.

    Returns
    -------
    list of list
        Toolpath segments. Each segment is a list of points.
    """

    if stepdown <= 0:
        raise ValueError("stepdown must be positive")
    if stepover <= 0:
        raise ValueError("stepover must be positive")
    if not boundary:
        return []

    cx = sum(p[0] for p in boundary) / len(boundary)
    cy = sum(p[1] for p in boundary) / len(boundary)
    max_r = max(math.hypot(p[0] - cx, p[1] - cy) for p in boundary)

    toolpath: List[List[Tuple[float, float, float]]] = []

    def _spiral_layer(z_level: float, so: float) -> None:
        r = max_r
        angle = 0.0
        phi_rad = math.radians(engagement_angle)
        while r > so:
            start = (
                cx + r * math.cos(math.radians(angle)),
                cy + r * math.sin(math.radians(angle)),
                z_level,
            )
            # tangential step derived from desired engagement angle
            tangential = so / math.tan(phi_rad)
            delta_deg = math.degrees(tangential / r)
            r2 = max(r - so, 0.0)
            angle2 = angle + delta_deg
            end = (
                cx + r2 * math.cos(math.radians(angle2)),
                cy + r2 * math.sin(math.radians(angle2)),
                z_level,
            )
            center = (cx, cy, z_level)
            toolpath.append([start, center, end])
            logging.info("Arc from %s to %s at depth %.3f", start, end, z_level)
            angle = angle2
            r = r2

    def _zigzag_layer(z_level: float, so: float) -> None:
        x_min = min(p[0] for p in boundary)
        x_max = max(p[0] for p in boundary)
        y_min = min(p[1] for p in boundary)
        y_max = max(p[1] for p in boundary)
        y = y_max
        direction = 1
        while y >= y_min:
            if direction == 1:
                start = (x_min, y, z_level)
                end = (x_max, y, z_level)
            else:
                start = (x_max, y, z_level)
                end = (x_min, y, z_level)
            toolpath.append([start, end])
            logging.info("Line from %s to %s at depth %.3f", start, end, z_level)
            y -= so
            direction *= -1

    layer_func = _spiral_layer if mode == "spiral" else _zigzag_layer

    current_z = 0.0
    layers = int(math.ceil(abs(depth) / stepdown))
    cur_stepdown = stepdown
    cur_stepover = stepover
    for i in range(layers):
        layer_func(current_z, cur_stepover)
        if adaptive_mode:
            cur_stepdown, cur_stepover = compute_adaptive_steps(
                cur_stepdown,
                cur_stepover,
                material_hardness=material_hardness,
                geometry_factor=geometry_factor,
                tool_load=tool_load,
            )
        current_z -= cur_stepdown
    if current_z > depth - 1e-6:
        layer_func(depth, cur_stepover)

    return toolpath

def generate_spiral_helix(
    center: Tuple[float, float],
    radius: float,
    depth: float,
    pitch: float,
    *,
    angle_step: float = 5.0,
) -> List[Tuple[float, float, float]]:
    """Generate a helical spiral toolpath.

    The tool descends while moving around ``center`` at constant ``radius``.
    ``pitch`` defines Z drop per full revolution. Negative ``depth`` means
    downward machining.
    """
    if radius <= 0:
        raise ValueError("radius must be positive")
    if pitch <= 0:
        raise ValueError("pitch must be positive")

    points: List[Tuple[float, float, float]] = []
    total_turns = abs(depth) / pitch
    steps = int(total_turns * 360.0 / angle_step) + 1
    for i in range(steps + 1):
        angle = i * angle_step
        z = -pitch * angle / 360.0
        if depth < 0 and z < depth:
            z = depth
        if depth > 0 and z > depth:
            z = depth
        rad = math.radians(angle)
        x = center[0] + radius * math.cos(rad)
        y = center[1] + radius * math.sin(rad)
        points.append((x, y, z))
        if z == depth:
            break
    logging.info(
        "Generated helical path radius %.3f depth %.3f with %d points",
        radius,
        depth,
        len(points),
    )
    return points


def morph_between_curves(
    curves: List[List[Tuple[float, float, float]]],
    layers: int,
) -> List[List[Tuple[float, float, float]]]:
    """Morph smoothly between two or more boundary curves.

    Parameters
    ----------
    curves : list of list
        Ordered boundary curves ``[(x, y, z), ...]``. At least two are required.
    layers : int
        Number of intermediate curves to generate between each pair of boundaries.

    Returns
    -------
    list of list
        Sequence of morphed curves including the originals.
    """

    if len(curves) < 2:
        raise ValueError("At least two curves are required")
    if layers < 1:
        raise ValueError("layers must be positive")

    def _resample(curve: List[Tuple[float, float, float]], n: int) -> List[Tuple[float, float, float]]:
        if len(curve) == n:
            return curve
        lengths = []
        total = 0.0
        for a, b in zip(curve[:-1], curve[1:]):
            seg_len = math.dist(a[:2], b[:2])
            total += seg_len
            lengths.append(total)
        steps = [i * total / (n - 1) for i in range(n)]
        resampled = [curve[0]]
        idx = 0
        for s in steps[1:]:
            while idx < len(lengths) - 1 and s > lengths[idx]:
                idx += 1
            prev_len = lengths[idx - 1] if idx else 0.0
            t = 0.0 if lengths[idx] == prev_len else (s - prev_len) / (lengths[idx] - prev_len)
            a = curve[idx]
            b = curve[idx + 1]
            pt = (
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
            )
            resampled.append(pt)
        return resampled

    point_count = max(len(c) for c in curves)
    resampled_curves = [_resample(c, point_count) for c in curves]

    result: List[List[Tuple[float, float, float]]] = [resampled_curves[0]]

    for i in range(len(resampled_curves) - 1):
        c0 = resampled_curves[i]
        c1 = resampled_curves[i + 1]
        for j in range(1, layers + 1):
            t = j / (layers + 1)
            seg = []
            for p0, p1 in zip(c0, c1):
                seg.append(
                    (
                        p0[0] * (1 - t) + p1[0] * t,
                        p0[1] * (1 - t) + p1[1] * t,
                        p0[2] * (1 - t) + p1[2] * t,
                    )
                )
            result.append(seg)
        result.append(c1)

    logging.info("Generated %d morphed curves", len(result))
    return result
