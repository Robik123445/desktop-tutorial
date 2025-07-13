"""Feedrate adjustments based on surface analysis."""

import logging
from pathlib import Path
from typing import List, Optional

from cam_slicer.utils import ZMap
from cam_slicer.logging_config import setup_logging

setup_logging()


def _parse_move(line: str):
    """Extract X, Y, Z and F values from a G-code move line."""
    x = y = z = None
    f = None
    for token in line.split():
        if token.startswith("X"):
            try:
                x = float(token[1:])
            except ValueError:
                pass
        elif token.startswith("Y"):
            try:
                y = float(token[1:])
            except ValueError:
                pass
        elif token.startswith("Z"):
            try:
                z = float(token[1:])
            except ValueError:
                pass
        elif token.startswith("F"):
            try:
                f = float(token[1:])
            except ValueError:
                pass
    return x, y, z, f


def optimize_toolpath_based_on_surface(
    gcode_path: str,
    heightmap_path: str,
    *,
    output_path: Optional[str] = None,
    min_depth: float = -0.5,
    rough_threshold: float = 0.2,
    feedrate_scale: float = 0.8,
) -> str:
    """Lower feedrate when cutting too deep or over rough regions.

    Parameters
    ----------
    gcode_path : str
        Path to input G-code file.
    heightmap_path : str
        File with probed heights (JSON or CSV).
    min_depth : float
        Depth difference triggering feed reduction.
    rough_threshold : float
        Roughness triggering additional reduction.
    feedrate_scale : float
        Scale factor applied to feedrate when conditions are met.

    Returns
    -------
    str
        Path to the optimized G-code file.

    Example
    -------
    >>> optimize_toolpath_based_on_surface("path.gcode", "probe.json")
    'path_optimized.gcode'
    """
    lines = Path(gcode_path).read_text().splitlines()
    zmap = ZMap.load(heightmap_path)
    optimized: List[str] = []
    prev_offset = None
    for line in lines:
        cmd = line.strip().split()[0] if line.strip() else ""
        if cmd in {"G0", "G1"}:
            x, y, z, fr = _parse_move(line)
            if x is None or y is None or z is None:
                optimized.append(line)
                continue
            offset = zmap.get_offset(x, y)
            pressure = z - offset
            roughness = 0.0
            if prev_offset is not None:
                roughness = abs(offset - prev_offset)
            prev_offset = offset
            new_fr = fr
            if fr is not None and (pressure < min_depth or roughness > rough_threshold):
                new_fr = fr * feedrate_scale
                logging.info(
                    "Lowering feedrate %.3f -> %.3f at X%.3f Y%.3f due to pressure %.3f roughness %.3f",
                    fr,
                    new_fr,
                    x,
                    y,
                    pressure,
                    roughness,
                )
            if new_fr is not None:
                parts = [p for p in line.split() if not p.startswith("F")]
                parts.append(f"F{new_fr:.3f}")
                line = " ".join(parts)
        optimized.append(line)

    if output_path is None:
        base = Path(gcode_path)
        output_path = str(base.with_name(base.stem + "_optimized" + base.suffix))

    Path(output_path).write_text("\n".join(optimized))
    logging.info("Optimized G-code saved to %s", output_path)
    return output_path
