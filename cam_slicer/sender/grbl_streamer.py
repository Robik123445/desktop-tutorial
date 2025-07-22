import logging
import time
from pathlib import Path
from typing import Optional

try:
    import serial
except ModuleNotFoundError as exc:  # pragma: no cover - optional dependency
    logger.warning("pyserial not installed: %s", exc)
    serial = None  # type: ignore

from .serial_streamer import _wait_for_ok


class StreamController:
    """Simple state holder for pause/resume/stop."""

    def __init__(self) -> None:
        self._paused = False
        self._stop = False

    def pause(self) -> None:
        self._paused = True
        logging.info("Streaming paused")

    def resume(self) -> None:
        self._paused = False
        logging.info("Streaming resumed")

    def stop(self) -> None:
        self._stop = True
        logging.info("Streaming stopped by user")


def stream_gcode_interactive(
    gcode_path: str,
    port: str,
    *,
    baud: int = 115200,
    controller: Optional[StreamController] = None,
) -> None:
    """Stream G-code with pause/resume/stop controls."""

    if serial is None:
        raise ImportError("pyserial is required for streaming")

    path = Path(gcode_path)
    if not path.is_file():
        raise FileNotFoundError(path)

    ctrl = controller or StreamController()

    with serial.Serial(port, baud, timeout=1) as ser:
        lines = path.read_text().splitlines()
        idx = 0
        while idx < len(lines):
            if ctrl._stop:
                break
            if ctrl._paused:
                time.sleep(0.1)
                continue
            line = lines[idx].strip()
            if not line:
                idx += 1
                continue
            ser.write((line + "\n").encode())
            _wait_for_ok(ser)
            idx += 1
