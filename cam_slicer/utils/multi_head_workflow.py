import logging
from typing import Iterable, Dict, Any, List

from cam_slicer.logging_config import setup_logging
from cam_slicer.core.header_footer import ControllerConfig
from cam_slicer.config.macros import get_macro

setup_logging()
logger = logging.getLogger(__name__)


def schedule_multi_head_job(
    segments: Iterable[Dict[str, Any]],
    controller_config: ControllerConfig,
    *,
    head_macros: Dict[str, str] | None = None,
) -> List[str]:
    """Generate G-code for multiple heads with automatic tool changes.

    Parameters
    ----------
    segments : iterable of dict
        Each segment must contain ``path`` and ``head`` keys.
    controller_config : ControllerConfig
        Controller settings for G-code formatting.
    head_macros : mapping, optional
        Mapping of head name to macro identifier inserted when switching heads.

    Returns
    -------
    list of str
        Combined G-code lines with tool changes.
    """
    from cam_slicer.core.gcode_export import toolpath_to_gcode  # lazy import to avoid circular deps

    seg_list = list(segments)
    if not seg_list:
        return []

    gcode: List[str] = []
    current_head = None
    total = len(seg_list)
    for idx, seg in enumerate(seg_list):
        head = seg.get("head", "default")
        lines = toolpath_to_gcode(
            seg.get("path", []),
            controller_config,
            feedrate=seg.get("feedrate"),
            laser_power=seg.get("laser_power"),
        )
        if idx > 0:
            lines = lines[1:]  # drop header
        if idx < total - 1:
            lines = lines[:-1]  # drop footer
        if head != current_head:
            logger.info("Switching to head %s", head)
            if idx == 0:
                gcode.append(lines[0])  # header
                lines = lines[1:]
            if head_macros and head in head_macros:
                try:
                    gcode.extend(get_macro(head_macros[head]))
                except KeyError:
                    logger.error("Macro %s not found", head_macros[head])
            else:
                gcode.append(f"; TOOL CHANGE {head}")
            current_head = head
        gcode.extend(lines)
    return gcode


def prepare_hybrid_job(
    paths: List[Iterable[Any]],
    controller_config: ControllerConfig,
    operations: Dict[int, str],
    tool_heads: List[str],
    *,
    operation_head_map: Dict[str, str] | None = None,
    head_macros: Dict[str, str] | None = None,
) -> tuple[list[str], dict[str, int]]:
    """Assign operations and heads then schedule into a single G-code job.

    Parameters
    ----------
    paths : list of iterables
        Vector paths for machining.
    controller_config : ControllerConfig
        Controller settings used for formatting.
    operations : mapping
        Mapping from path index to operation name.
    tool_heads : list of str
        Available heads, e.g. ``["router", "laser"]``.
    operation_head_map : mapping, optional
        Custom operation to head mapping.
    head_macros : mapping, optional
        Tool change macros inserted when switching heads.

    Returns
    -------
    tuple
        ``(gcode_lines, summary)`` where ``summary`` counts segments per head.
    """

    from .hybrid_operations import assign_hybrid_operations

    segments, summary = assign_hybrid_operations(
        paths, tool_heads, operations, operation_head_map
    )
    gcode = schedule_multi_head_job(
        segments, controller_config, head_macros=head_macros
    )
    return gcode, summary
