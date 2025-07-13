"""3+2 indexed positioning utilities for robotic machining."""

from __future__ import annotations

import logging
from typing import Iterable, List, Sequence, Tuple

from cam_slicer.logging_config import setup_logging
from cam_slicer.core.gcode_export import toolpath_to_gcode
from cam_slicer.core.header_footer import ControllerConfig

setup_logging()
logger = logging.getLogger(__name__)


def _apply_orientation(
    points: Iterable[Sequence[float]],
    orientation: Tuple[float, float, float],
) -> List[Tuple[float, float, float, float, float, float]]:
    """Attach orientation angles to each point."""
    a, b, c = orientation
    oriented = []
    for p in points:
        x, y, z = (list(p) + [0.0, 0.0, 0.0])[:3]
        oriented.append((x, y, z, a, b, c))
    return oriented


def generate_indexed_gcode(
    toolpath: List[Sequence[float]],
    orientations: List[Tuple[float, float, float]],
    controller_config: ControllerConfig,
    *,
    segment_length: int | None = None,
    transform_cfg=None,
    feedrate: float | None = None,
) -> List[str]:
    """Generate G-code for 3+2 indexed machining.

    Parameters
    ----------
    toolpath : list of (x, y, z)
        Cartesian toolpath points.
    orientations : list of (a, b, c)
        Fixed orientations in degrees. Each orientation is applied to a
        consecutive segment of the toolpath.
    controller_config : ControllerConfig
        Target controller configuration for header/footer and units.
    segment_length : int, optional
        Number of points per orientation segment. If ``None`` segments are
        divided evenly.
    transform_cfg : TransformConfig, optional
        Additional transformation applied to each point before exporting.
    feedrate : float, optional
        Feedrate for linear moves.

    Returns
    -------
    list of str
        Generated G-code lines with orientation moves.
    """
    if not toolpath or not orientations:
        return []

    n = len(toolpath)
    if segment_length is None:
        segment_length = max(1, n // len(orientations))

    gcode: List[str] = []
    idx = 0
    for i, orient in enumerate(orientations):
        seg = toolpath[idx : idx + segment_length]
        if not seg:
            break
        idx += segment_length
        gcode.append(f"G0 A{orient[0]:.3f} B{orient[1]:.3f} C{orient[2]:.3f}")
        seg_oriented = _apply_orientation(seg, orient)
        lines = toolpath_to_gcode(
            seg_oriented,
            controller_config,
            transform_cfg,
            feedrate=feedrate,
        )
        gcode.extend(lines[1:-1])  # skip header/footer for inner segments
    footer = toolpath_to_gcode([], controller_config)[-1]
    header = toolpath_to_gcode([], controller_config)[0]
    gcode.insert(0, header)
    gcode.append(footer)
    logger.info("Generated %d indexed segments", len(orientations))
    return gcode

