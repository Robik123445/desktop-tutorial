import logging
import re
from typing import Callable, Tuple, Optional

from cam_slicer.logging_config import setup_logging
from cam_slicer.sensors import generate_heightmap, export_heightmap_to_json
from cam_slicer.utils.zmap import ZMap

setup_logging()
logger = logging.getLogger(__name__)

try:
    import serial  # type: ignore
except ModuleNotFoundError as exc:  # pragma: no cover - optional
    logger.warning("pyserial not installed: %s", exc)
    serial = None  # type: ignore


def _default_send(cmd: str, port: "serial.Serial") -> str:
    """Send a command and return the device response."""
    port.write((cmd + "\n").encode())
    logger.info("Sent: %s", cmd)
    resp = port.readline().decode().strip()
    if resp:
        logger.info("Recv: %s", resp)
    return resp


def _probe_using_serial(x: float, y: float, port: "serial.Serial", depth: float, feed: float) -> float:
    """Move to ``(x, y)`` and probe Z using G38.2."""
    _default_send(f"G0 X{x:.3f} Y{y:.3f}", port)
    resp = _default_send(f"G38.2 Z{depth:.3f} F{feed:.1f}", port)
    m = re.search(r"-?\d+(?:\.\d+)?", resp)
    z = float(m.group(0)) if m else 0.0
    _default_send("G0 Z5", port)
    return z


def probe_heightmap(
    x_range: Tuple[float, float],
