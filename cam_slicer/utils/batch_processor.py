import logging
from typing import Iterable, Sequence, List

from ..logging_config import setup_logging
from ..config.macros import get_macro

setup_logging()
logger = logging.getLogger(__name__)


def run_macro_sequence(macro_names: Iterable[str]) -> List[str]:
    """Return list of G-code lines for the given macro names."""
    lines: List[str] = []
    for name in macro_names:
        try:
            lines.extend(get_macro(name))
        except KeyError as exc:  # pragma: no cover - log only
            logger.error("Macro %s not found: %s", name, exc)
    logger.info("Loaded %d macros", len(macro_names))
    return lines


def batch_process_toolpaths(
    toolpaths: Iterable[Iterable[Sequence[float]]],
    controller_config: "ControllerConfig",
    *,
    start_macro: str | None = None,
    between_macro: str | None = None,
    end_macro: str | None = None,
) -> List[str]:
    """Combine multiple toolpaths into one G-code list with optional macros."""
    from ..core.gcode_export import toolpath_to_gcode, ControllerConfig as _CC
    assert isinstance(controller_config, _CC)
    paths = list(toolpaths)
    gcode: List[str] = []
    for idx, tp in enumerate(paths):
        lines = toolpath_to_gcode(
            tp,
            controller_config,
            start_macro=start_macro if idx == 0 else None,
            end_macro=end_macro if idx == len(paths) - 1 else None,
        )
        if idx > 0:
            lines = lines[1:]  # drop header
        if idx < len(paths) - 1:
            lines = lines[:-1]  # drop footer
        gcode.extend(lines)
        if between_macro and idx < len(paths) - 1:
            gcode.extend(run_macro_sequence([between_macro]))
    logger.info("Batch processed %d toolpaths", len(paths))
    return gcode
