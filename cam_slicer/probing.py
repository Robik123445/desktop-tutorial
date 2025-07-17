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
except ModuleNotFoundError:  # pragma: no cover - optional
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
    y_range: Tuple[float, float],
    step: float,
    *,
    port: Optional[str] = None,
    baud: int = 115200,
    probe_depth: float = -2.0,
    feed: float = 100.0,
    probe_func: Optional[Callable[[float, float], float]] = None,
    save_path: Optional[str] = None,
) -> ZMap:
    """Probe a grid and return a :class:`ZMap` with measured heights.

    If ``probe_func`` is provided, it will be used to obtain Z values for each
    ``(x, y)`` pair. Otherwise ``port`` must be specified and ``pyserial``
    available so probing commands are sent to the machine.
    """

    if probe_func is None:
        if port is None:
            raise ValueError("port required when probe_func is None")
        if serial is None:
            raise ImportError("pyserial is required for probing")
        ser = serial.Serial(port, baud, timeout=1)

        def probe_func(x: float, y: float) -> float:
            return _probe_using_serial(x, y, ser, probe_depth, feed)

    hm_dict = generate_heightmap(x_range, y_range, step, probe_func)
    points = [(x, y, z) for (x, y), z in hm_dict.items()]
    zmap = ZMap(points)

    if save_path:
        export_heightmap_to_json(hm_dict, save_path)
    logger.info("Probed %d points", len(points))

    if probe_func is not None and port is not None and serial is not None:
        ser.close()

    return zmap
