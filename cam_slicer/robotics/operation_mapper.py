import logging
import os
from typing import Dict, Iterable

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


def map_operations(
    file_paths: Iterable[str],
    overrides: Dict[str, str] | None = None,
) -> Dict[str, str]:
    """Map input files to operations.

    Parameters
    ----------
    file_paths : Iterable[str]
        Paths to imported files.
    overrides : dict, optional
        Mapping from file path to operation type to override heuristic.

    Returns
    -------
    dict
        Mapping of file path to operation (cut, engrave, mill, print).
    """
    overrides = overrides or {}
    result: Dict[str, str] = {}
    for path in file_paths:
        if path in overrides:
            result[path] = overrides[path]
            logger.info("Manual operation for %s: %s", path, overrides[path])
            continue
        ext = os.path.splitext(path)[1].lower()
        if ext in {".svg", ".dxf", ".dwg"}:
            op = "cut"
        elif ext in {".stl", ".obj"}:
            op = "mill"
        elif ext in {".gcode"}:
            op = "print"
        else:
            op = "engrave"
        result[path] = op
        logger.debug("Mapped %s to %s", path, op)
    return result
