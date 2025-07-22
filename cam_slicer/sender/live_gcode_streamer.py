import logging
import time
from pathlib import Path

try:
    import serial
except ModuleNotFoundError as exc:  # pragma: no cover - optional dependency
    logging.warning("pyserial not installed: %s", exc)
    serial = None  # type: ignore


class LiveGcodeStreamer:
    """Stream G-code to GRBL with pause/resume/stop."""

    def __init__(self, gcode_path: str, port: str, baud: int = 115200) -> None:
        if serial is None:
            raise ImportError("pyserial is required for streaming")
        self.path = Path(gcode_path)
        if not self.path.is_file():
            raise FileNotFoundError(self.path)
        self.port = port
        self.baud = baud
        self._paused = False
        self._stop = False
        logging.info("LiveGcodeStreamer prepared for %s", self.path)

    def pause(self) -> None:
        """Pause sending commands."""
        self._paused = True
        logging.info("Streaming paused")

    def resume(self) -> None:
        """Resume sending commands."""
        self._paused = False
        logging.info("Streaming resumed")

    def stop(self) -> None:
        """Stop streaming."""
        self._stop = True
        logging.info("Streaming stopped by user")

    def stream(self) -> None:
        """Send the configured G-code file."""
        if serial is None:
            raise ImportError("pyserial is required for streaming")

        with serial.Serial(self.port, self.baud, timeout=1) as ser:
            lines = self.path.read_text().splitlines()
            idx = 0
            while idx < len(lines):
                if self._stop:
                    break
                if self._paused:
                    time.sleep(0.1)
                    continue
                line = lines[idx].strip()
                if not line:
                    idx += 1
                    continue
                ser.write((line + "\n").encode())
                _wait_for_ok(ser)
                idx += 1
