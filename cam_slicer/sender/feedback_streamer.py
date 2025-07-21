import logging
from pathlib import Path
from typing import Callable

from cam_slicer.logging_config import setup_logging
from .serial_streamer import _wait_for_ok

setup_logging()
logger = logging.getLogger(__name__)

try:
    import serial  # type: ignore
except ModuleNotFoundError as exc:  # pragma: no cover - optional dependency
    logger.warning("pyserial not installed: %s", exc)
    serial = None  # type: ignore


def _parse_xyz(line: str) -> tuple[float, float, float]:
    """Extract X, Y, Z from a G-code line if present."""
    x = y = z = 0.0
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
    return x, y, z
