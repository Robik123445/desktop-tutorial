import logging
from typing import List, Dict, Iterable, Tuple, Any, Callable

from cam_slicer.logging_config import setup_logging

setup_logging()


def assign_hybrid_operations(
    paths: List[Iterable[Any]],
    tool_heads: List[str],
    path_operations: Dict[int, str],
    operation_head_map: Dict[str, str] | None = None,
) -> Tuple[List[Dict[str, Any]], Dict[str, int]]:
    """Assign operations and tool heads to vector paths.

    Parameters
    ----------
    paths: list of vector paths where each path is iterable of points.
    tool_heads: available tool heads, e.g. ["router", "laser"].
    path_operations: mapping from path index to operation name.
    operation_head_map: optional mapping from operation to tool head which
        overrides the defaults {"cut": "router", "engrave": "laser", "drill": "engraver"}.

    Returns
    -------
    segments: list of dictionaries with keys ``path``, ``operation`` and ``head``.
    summary: number of segments assigned to each head.
    """

    default_map = {
        "cut": "router",
        "engrave": "laser",
        "drill": "engraver",
        "print": "print_head",
        "mill": "router",
        "laser_cut": "laser",
        "laser_engrave": "laser",
    }
    if operation_head_map:
        default_map.update(operation_head_map)

    segments: List[Dict[str, Any]] = []
    summary: Dict[str, int] = {}

    if not tool_heads:
        logging.error("No tool heads available")
        return segments, summary

    for idx, path in enumerate(paths):
        operation = path_operations.get(idx, "cut")
        head = default_map.get(operation, default_map["cut"])
        if head not in tool_heads:
            logging.warning("Head %s not available, falling back to %s", head, tool_heads[0])
            head = tool_heads[0]
        segments.append({"path": list(path), "operation": operation, "head": head})
        summary[head] = summary.get(head, 0) + 1

    logging.info("Assigned %d segments: %s", len(segments), summary)
    return segments, summary


def blend_additive_subtractive(
    additive_paths: List[Iterable[Any]],
    subtractive_paths: List[Iterable[Any]],
    controller_config: "ControllerConfig",
    *,
    head_macros: Dict[str, str] | None = None,
    operation_head_map: Dict[str, str] | None = None,
) -> tuple[list[str], dict[str, int]]:
    """Combine additive and subtractive paths into one hybrid job.

    Parameters
    ----------
    additive_paths : list of iterables
        3D printing toolpaths.
    subtractive_paths : list of iterables
        Milling toolpaths.
    controller_config : ControllerConfig
        Controller settings used for G-code formatting.
    head_macros : dict, optional
        Tool change macros keyed by head name.
    operation_head_map : dict, optional
        Custom mapping of operation to tool head.

    Returns
    -------
    tuple
        ``(gcode_lines, summary)`` for the combined workflow.
    """

    from .multi_head_workflow import prepare_hybrid_job

    paths: List[Iterable[Any]] = []
    operations: Dict[int, str] = {}

    idx = 0
    for p in additive_paths:
        paths.append(p)
        operations[idx] = "print"
        idx += 1

    for p in subtractive_paths:
        paths.append(p)
        operations[idx] = "mill"
        idx += 1

    tool_heads = ["print_head", "router"]
    return prepare_hybrid_job(
        paths,
        controller_config,
        operations,
        tool_heads,
        operation_head_map=operation_head_map,
        head_macros=head_macros,
    )


def blend_laser_mill(
    laser_paths: List[Iterable[Any]],
    mill_paths: List[Iterable[Any]],
    controller_config: "ControllerConfig",
    *,
    laser_power: float = 1000.0,
    feedrate_laser: float = 800.0,
    feedrate_mill: float = 500.0,
    head_macros: Dict[str, str] | None = None,
) -> tuple[list[str], dict[str, int]]:
    """Combine laser and milling paths into one hybrid job with power control."""

    from .multi_head_workflow import schedule_multi_head_job

    segments: List[Dict[str, Any]] = []
    summary: Dict[str, int] = {}

    for p in laser_paths:
        segments.append(
            {
                "path": list(p),
                "head": "laser",
                "operation": "laser_engrave",
                "feedrate": feedrate_laser,
                "laser_power": laser_power,
            }
        )
        summary["laser"] = summary.get("laser", 0) + 1

    for p in mill_paths:
        segments.append(
            {
                "path": list(p),
                "head": "router",
                "operation": "mill",
                "feedrate": feedrate_mill,
            }
        )
        summary["router"] = summary.get("router", 0) + 1

    gcode = schedule_multi_head_job(segments, controller_config, head_macros=head_macros)
    return gcode, summary
