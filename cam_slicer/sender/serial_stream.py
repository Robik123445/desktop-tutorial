"""Basic serial streaming helpers."""

from __future__ import annotations

import logging
from pathlib import Path

from cam_slicer.logging_config import setup_logging

setup_logging()

try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    serial = None  # type: ignore

try:
    from serial.tools import list_ports  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    list_ports = None  # type: ignore


def _read_until_ok(port: "serial.Serial") -> str:
    """Read a line from port and return it.

    The function waits until a non-empty response is received.
    The trailing newline is stripped and the line is logged.
    """
    if serial is None:
        raise ImportError("pyserial is required for streaming")

    while True:
        resp = port.readline().decode().strip()
        if resp:
            logging.info("Received: %s", resp)
            return resp


def stream_gcode_live(gcode_path: str | Path, port: str, baud: int = 115200) -> None:
    """Stream gcode_path line by line and wait for ok after each command."""
    if serial is None:
        raise ImportError("pyserial is required for streaming")

    path = Path(gcode_path)
    if not path.is_file():
        raise FileNotFoundError(path)

    lines = [ln.strip() for ln in path.read_text().splitlines() if ln.strip()]
    with serial.Serial(port, baud, timeout=1) as ser:
        logging.info("Streaming %s to %s at %d baud", path, port, baud)
        for cmd in lines:
            ser.write((cmd + "\n").encode())
            logging.info("Sent: %s", cmd)
            resp = _read_until_ok(ser)
            if resp.lower() != "ok":
                logging.error("Controller responded with: %s", resp)
                raise RuntimeError(f"Unexpected response: {resp}")
    logging.info("Finished streaming %s", path)


def send_gcode_over_serial(gcode_text: str, port: str, baud: int = 115200) -> str:
    """Send raw G-code commands and capture responses."""
    if serial is None:
        raise ImportError("pyserial is required for streaming")

    lines = [ln.strip() for ln in gcode_text.splitlines() if ln.strip()]
    log_lines = []
    with serial.Serial(port, baud, timeout=1) as ser:
        for cmd in lines:
            ser.write((cmd + "\n").encode())
            logging.info("Sent: %s", cmd)
            resp = ser.readline().decode().strip()
            if resp:
                logging.info("Received: %s", resp)
            log_lines.append(f"sent: {cmd}")
            if resp:
                log_lines.append(f"recv: {resp}")
    return "\n".join(log_lines)


def list_available_ports() -> list[str]:
    """Return a list of available serial port device names."""
    if list_ports is None:
        raise ImportError("pyserial is required for listing ports")
    return [p.device for p in list_ports.comports()]
