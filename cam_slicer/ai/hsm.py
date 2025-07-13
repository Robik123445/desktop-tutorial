"""High-speed machining (HSM) utilities."""

from __future__ import annotations

import logging
from typing import Iterable, List, Tuple, Union, Dict

from cam_slicer.logging_config import setup_logging
from cam_slicer.robotics.kinematics import (
    calculate_junction_velocity,
    plan_feedrate_with_lookahead,
)
from cam_slicer.core.gcode_export import (
    toolpath_to_gcode,
    ControllerConfig,
    _arc_command,
    _get_header_footer,
)
from cam_slicer.ai.trajectory_planner import smooth_toolpath_corners

setup_logging()


Move = Tuple[float, float, float]
Arc = Tuple[str, Dict[str, object]]


def filter_small_arcs(
    moves: List[Union[Move, Arc]], radius_threshold: float = 0.5
) -> List[Union[Move, Arc]]:
    """Remove arc instructions with radius below ``radius_threshold``."""
    filtered: List[Union[Move, Arc]] = []
    for item in moves:
        if isinstance(item, tuple) and item and item[0] == "arc":
            arc = item[1]
            if arc.get("radius", 0.0) < radius_threshold:
                logging.info(
                    "Filtering small arc r=%.3f at %s",
                    arc.get("radius", 0.0),
                    arc.get("start"),
                )
                # replace arc by straight move to end
                filtered.append(arc["end"])
                continue
        filtered.append(item)
    return filtered


def optimize_high_speed_toolpath(
    toolpath: Iterable[Move],
    controller_config: ControllerConfig,
    *,
    feedrate: float = 1000.0,
    lookahead: int = 5,
    max_acceleration: float = 1500.0,
    junction_deviation: float = 0.05,
    corner_angle: float = 30.0,
    arc_radius_threshold: float = 0.5,
) -> List[str]:
    """Generate HSM-optimized G-code from raw toolpath."""
    path = list(toolpath)
    if len(path) < 2:
        return toolpath_to_gcode(path, controller_config, feedrate=feedrate)

    smoothed = smooth_toolpath_corners(path, angle_threshold=corner_angle)
    smoothed = filter_small_arcs(smoothed, radius_threshold=arc_radius_threshold)

    # flatten arcs back to points for feedrate planning
    flat: List[Move] = []
    for item in smoothed:
        if isinstance(item, tuple) and item and item[0] == "arc":
            arc = item[1]
            flat.extend([arc["start"], arc["center"], arc["end"]])
        else:
            flat.append(item)  # type: ignore

    feedrates = plan_feedrate_with_lookahead(
        flat, feedrate, max_acceleration, junction_deviation, queue_size=lookahead
    )

    result: List[str] = []
    fr_iter = iter(feedrates)
    for item in smoothed:
        fr = next(fr_iter, feedrate)
        if isinstance(item, tuple) and item and item[0] == "arc":
            arc = item[1]
            line = _arc_command(arc["start"], arc["center"], arc["end"])
            line += f" F{fr:.3f}"
            result.append(line)
            # consume two extra feedrates for center and end
            next(fr_iter, None)
            next(fr_iter, None)
        else:
            x, y, z = item  # type: ignore
            line = f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F{fr:.3f}"
            result.append(line)
    header, footer = _get_header_footer(controller_config)
    return [header] + result + [footer]
