import logging
from pathlib import Path
from typing import Any, Iterable, List, Tuple

from cam_slicer.logging_config import setup_logging
from cam_slicer.importers import import_stl, import_svg, import_dwg
from cam_slicer.core.gcode_export import toolpath_to_gcode
from cam_slicer.core.header_footer import ControllerConfig

setup_logging()


def import_model(path: str) -> Any:
    """Load a CAD file and return geometry.

    The loader picks the right parser based on extension. Supported
    formats are STL, SVG and DXF/DWG.
    """
    ext = Path(path).suffix.lower()
    logging.info("Importing model %s", path)
    if ext == ".stl":
        return import_stl(path)
    if ext == ".svg":
        return import_svg(path)
    if ext in {".dxf", ".dwg"}:
        return import_dwg(path)
    raise ValueError(f"Unsupported format: {ext}")


def export_gcode(toolpath: Iterable[Tuple[float, float, float]],
                 controller: str = "grbl") -> str:
    """Convert a toolpath to G-code text."""
    cfg = ControllerConfig(CONTROLLER_TYPE=controller)
    lines = toolpath_to_gcode(list(toolpath), cfg)
    logging.info("Exported %d G-code lines", len(lines))
    return "\n".join(lines)
