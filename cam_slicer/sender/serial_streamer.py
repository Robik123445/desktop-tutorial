"""G-code streaming specifically for GRBL controllers."""

import logging
from pathlib import Path
from typing import Optional

from cam_slicer.logging_config import setup_logging
setup_logging()
try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    serial = None  # type: ignore

# Configure basic logging


def _wait_for_ok(port: "serial.Serial", timeout: float = 2.0) -> None:
    """Wait for an ``ok`` response from GRBL.

    Parameters
    ----------
    port : serial.Serial
        Opened serial port.
    timeout : float, optional
        Maximum time to wait for a response.

    Raises
    ------
    RuntimeError
        If an ``error`` is received from the controller.
    TimeoutError
        If no response arrives within ``timeout`` seconds.
    """

    if serial is None:  # pragma: no cover - runtime check
        raise ImportError("pyserial is required for streaming")

    import time

    start = time.time()
    while True:
        resp = port.readline().decode().strip()
        if resp:
            logging.info("Received: %s", resp)
            low = resp.lower()
            if low.startswith("ok"):
                return
            if low.startswith("error"):
                raise RuntimeError(resp)
        if time.time() - start > timeout:
            logging.error("Timeout waiting for ok")
            raise TimeoutError("No response from controller")


def stream_gcode_to_grbl(filepath: str, port: str, baud: int = 115200) -> None:
    """Send a G-code file to a GRBL controller line by line.

    The function opens ``filepath`` and sends each command to the controller
    via the given serial ``port``. Transmission waits for the ``ok`` response
    before continuing with the next line.

    Parameters
    ----------
    filepath : str
        Path to the G-code file.
    port : str
        Serial port device, e.g. ``COM3`` or ``/dev/ttyUSB0``.
    baud : int, optional
        Baudrate for the connection, by default 115200.

    Raises
    ------
    RuntimeError
        When the controller returns ``error``.
    TimeoutError
        When the controller does not respond in time.
    FileNotFoundError
        If ``filepath`` does not exist.
    """

    if serial is None:
        raise ImportError("pyserial is required for streaming")

    path = Path(filepath)
    if not path.is_file():
        raise FileNotFoundError(path)

    with serial.Serial(port, baud, timeout=1) as ser, path.open("r", encoding="utf-8") as fh:
        for raw in fh:
            cmd = raw.strip()
            if not cmd:
                continue
            ser.write((cmd + "\n").encode())
            logging.info("Sent: %s", cmd)
            _wait_for_ok(ser)

    logging.info("Finished streaming %s", filepath)
