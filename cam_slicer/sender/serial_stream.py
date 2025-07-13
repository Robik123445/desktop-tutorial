"""Live G-code streaming over serial."""

from pathlib import Path
import logging
from cam_slicer.logging_config import setup_logging
setup_logging()
try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    serial = None  # type: ignore

# Common logging configuration


def _read_until_ok(port: "serial.Serial") -> str:
    """Read from serial until a line is received."""
    resp = port.readline().decode().strip()
    logging.info("Received: %s", resp)
    return resp


def stream_gcode_live(gcode_path: str | Path, port: str, baud: int) -> None:
    """Stream G-code line by line and wait for 'ok'.

    Parameters
    ----------
    gcode_path : str or Path
        Path to G-code file to send.
    port : str
        Serial port device, e.g. ``COM3`` or ``/dev/ttyUSB0``.
    baud : int
        Baudrate for the connection.

    Raises
    ------
    RuntimeError
        When controller response does not contain ``ok``.
    """

    if serial is None:
        raise ImportError("pyserial is required for streaming")

    path = Path(gcode_path)
    if not path.is_file():
        raise FileNotFoundError(path)

    with serial.Serial(port, baud, timeout=1) as ser, path.open("r", encoding="utf-8") as fh:
        for line in fh:
            cmd = line.strip()
            if not cmd:
                continue
            ser.write((cmd + "\n").encode())
            logging.info("Sent: %s", cmd)
            resp = _read_until_ok(ser)
            if resp.lower() != "ok":
                logging.error("Controller responded with: %s", resp)
                raise RuntimeError(f"Unexpected response: {resp}")

