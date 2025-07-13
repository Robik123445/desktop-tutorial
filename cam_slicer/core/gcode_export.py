"""G-code export utilities.

This module converts toolpath coordinates into G-code. It relies on
:func:`~cam_slicer.robotics.kinematics.transform_point` and related helpers.
Usage is demonstrated at the bottom of the file.
"""

from __future__ import annotations

import logging
import math
from typing import Iterable

from cam_slicer.logging_config import setup_logging
from cam_slicer.performance import profiled, parallel_map
from cam_slicer.robotics.kinematics import (
    transform_point,
    TransformConfig,
    apply_pressure_advance,
    calculate_junction_velocity,
)
from cam_slicer.config.macros import get_macro
from cam_slicer.utils import ZMap

from .header_footer import ControllerConfig, _get_header_footer

setup_logging()


def _transform_pt(args: tuple[tuple[float, ...], TransformConfig]):
    """Helper for parallel coordinate transformation."""
    p, cfg = args
    return transform_point(*p[:3], cfg)


def _arc_command(start: tuple[float, float, float],
                 center: tuple[float, float, float],
                 end: tuple[float, float, float]) -> str:
    """Create GRBL G2/G3 command from three points."""
    sx, sy, _ = start
    cx, cy, _ = center
    ex, ey, _ = end
    i = cx - sx
    j = cy - sy
    v1 = (sx - cx, sy - cy)
    v2 = (ex - cx, ey - cy)
    cross = v1[0] * v2[1] - v1[1] * v2[0]
    cmd = "G3" if cross > 0 else "G2"
    return f"{cmd} X{ex:.3f} Y{ey:.3f} I{i:.3f} J{j:.3f}"


@profiled
def toolpath_to_gcode(
    toolpath: Iterable[tuple[float, float, float]],
    controller_config: ControllerConfig,
    transform_cfg: TransformConfig | None = None,
    shape: str = "line",
    arc_support: bool = False,
    move_command: str = "G1",
    feedrate: float | None = None,
    laser_power: float | None = None,
    advance_factor: float = 0.02,
    lookahead: int = 0,
    max_acceleration: float = 1000.0,
    junction_deviation: float = 0.05,
    z_map: ZMap | None = None,
    adaptive_mode: bool = False,
    start_macro: str | None = None,
    mid_macro: str | None = None,
    end_macro: str | None = None,
    parallel: bool = False,
    workers: int | None = None,
) -> list[str]:
    """Convert a toolpath sequence into G-code.

    Parameters
    ----------
    laser_power : float, optional
        If provided, enable laser mode and set power via ``M3 S<power>`` / ``M5``
        commands.
    """
    header, footer = _get_header_footer(controller_config)
    gcode_lines = [header]
    if start_macro:
        gcode_lines.extend(get_macro(start_macro))
    if laser_power is not None:
        gcode_lines.append(f"M3 S{laser_power:.1f}")
    cfg = transform_cfg or TransformConfig()
    cmd = move_command.upper()
    prev_z = None
    lookahead_queue: list[tuple[float, float, float]] = []
    mid_index = len(toolpath) // 2 if mid_macro else None
    path = list(toolpath)
    if parallel:
        transformed = parallel_map(_transform_pt, [(p, cfg) for p in path], max_workers=workers)
    else:
        transformed = [transform_point(*p[:3], cfg) for p in path]

    if shape == "circle" and arc_support and len(path) >= 3:
        start, center, end = transformed[0], transformed[1], transformed[2]
        if z_map is not None:
            start = (start[0], start[1], start[2] + z_map.get_offset(*start[:2]))
            end = (end[0], end[1], end[2] + z_map.get_offset(*end[:2]))
        gcode_lines.append(_arc_command(start, center, end))
        if mid_macro:
            gcode_lines.extend(get_macro(mid_macro))
    else:
        n = len(transformed)
        for i, tpt in enumerate(transformed):
            x, y, z = tpt
            angles = path[i][3:] if len(path[i]) > 3 else []
            while lookahead > 0 and len(lookahead_queue) < lookahead + 1 and i + len(lookahead_queue) < n:
                lookahead_queue.append(transformed[i + len(lookahead_queue)])
            if z_map is not None and (prev_z is None or z != prev_z):
                z += z_map.get_offset(x, y)
            if cmd == "G1" and feedrate is not None:
                base_fr = feedrate
                if lookahead > 0 and len(lookahead_queue) >= 3:
                    min_jv = feedrate
                    for j in range(len(lookahead_queue) - 2):
                        prev_vec = tuple(
                            lookahead_queue[j + 1][k] - lookahead_queue[j][k]
                            for k in range(3)
                        )
                        next_vec = tuple(
                            lookahead_queue[j + 2][k] - lookahead_queue[j + 1][k]
                            for k in range(3)
                        )
                        jv = calculate_junction_velocity(
                            prev_vec,
                            next_vec,
                            max_acceleration,
                            junction_deviation,
                        )
                        if not math.isinf(jv):
                            min_jv = min(min_jv, jv)
                    base_fr = min(base_fr, min_jv)
                if adaptive_mode and z_map is not None:
                    offset = z_map.get_offset(x, y)
                    scale = max(0.5, 1.0 - abs(offset) * 0.1)
                    base_fr *= scale
                    logging.info("Adaptive mode: offset %.3f scales feed %.3f", offset, base_fr)
                adj_fr = apply_pressure_advance(base_fr, 500.0, advance_factor)
                logging.info("Pressure advance: %.3f -> %.3f", base_fr, adj_fr)
                line = f"{cmd} X{x:.3f} Y{y:.3f} Z{z:.3f}"
                if angles:
                    if len(angles) > 0:
                        line += f" A{angles[0]:.3f}"
                    if len(angles) > 1:
                        line += f" B{angles[1]:.3f}"
                    if len(angles) > 2:
                        line += f" C{angles[2]:.3f}"
                line += f" F{adj_fr:.3f}"
                gcode_lines.append(line)
                prev_z = z
            else:
                line = f"{cmd} X{x:.3f} Y{y:.3f} Z{z:.3f}"
                if angles:
                    if len(angles) > 0:
                        line += f" A{angles[0]:.3f}"
                    if len(angles) > 1:
                        line += f" B{angles[1]:.3f}"
                    if len(angles) > 2:
                        line += f" C{angles[2]:.3f}"
                gcode_lines.append(line)
                prev_z = z
            if mid_macro and i == mid_index:
                gcode_lines.extend(get_macro(mid_macro))
            if lookahead_queue:
                lookahead_queue.pop(0)
    if laser_power is not None:
        gcode_lines.append("M5")
    if end_macro:
        gcode_lines.extend(get_macro(end_macro))
    gcode_lines.append(footer)
    return gcode_lines


# Example usage
if __name__ == "__main__":  # pragma: no cover
    tp = [(0, 0, 0), (10, 0, 0)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    lines = toolpath_to_gcode(tp, cfg)
    for line in lines:
        print(line)
