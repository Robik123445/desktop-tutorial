import logging
from pathlib import Path
from typing import Callable

from cam_slicer.logging_config import setup_logging
from .serial_streamer import _wait_for_ok

setup_logging()

try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
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


def stream_gcode_with_feedback(
    filepath: str,
    port: str,
    measure_func: Callable[[float, float], float],
    *,
    baud: int = 115200,
    threshold: float = 0.05,
) -> None:
    """Stream G-code and adjust future Z moves using live measurements.

    Parameters
    ----------
    filepath : str
        Path to the G-code file.
    port : str
        Serial port device.
    measure_func : Callable[[float, float], float]
        Function returning measured Z value at given ``(x, y)``.
    baud : int, optional
        Serial baud rate. Default ``115200``.
    threshold : float, optional
        Minimum deviation in millimetres to trigger correction. Default ``0.05``.
    """
    if serial is None:
        raise ImportError("pyserial is required for streaming")

    path = Path(filepath)
    if not path.is_file():
        raise FileNotFoundError(path)

    lines = [ln.strip() for ln in path.read_text().splitlines() if ln.strip()]
    offset = 0.0

    with serial.Serial(port, baud, timeout=1) as ser:
        logging.info("Streaming with feedback to %s", port)
        for line in lines:
            if line.startswith("G"):
                x, y, z = _parse_xyz(line)
                if "Z" in line:
                    z += offset
                    line = f"{line.split('Z')[0]}Z{z:.3f}"  # replace Z
            ser.write((line + "\n").encode())
            logging.info("Sent: %s", line)
            _wait_for_ok(ser)

            if "G1" in line or "G0" in line:
                measured_z = measure_func(x, y)
                diff = measured_z - z
                if abs(diff) > threshold:
                    offset -= diff
                    logging.info(
                        "Measurement diff %.3f at X%.3f Y%.3f -> applying offset %.3f",
                        diff,
                        x,
                        y,
                        offset,
                    )

    logging.info("Finished streaming with feedback")
