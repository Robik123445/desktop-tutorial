import logging
from pathlib import Path

logger = logging.getLogger(__name__)
try:
    import serial  # type: ignore
except ModuleNotFoundError as exc:  # pragma: no cover - optional dependency
    logger.warning("pyserial not installed: %s", exc)
    serial = None  # type: ignore

def _read_until_ok(ser):
    """Read lines from serial until 'ok' is received."""
    line = ""
    while True:
        line = ser.readline().decode().strip()
        if not line:
            continue
        if "ok" in line.lower():
            break
    return line

def stream_gcode_live(gcode_path: str | Path, port: str, baud: int) -> None:
    """
    Stream G-code lines from a file to a serial port.

    Raises
    ------
    RuntimeError
        When controller response does not contain ``ok``.
    """

    if serial is None:
        raise ImportError("pyserial is required for streaming")
