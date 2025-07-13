"""Class-based G-code streamer with pause and stop controls."""

from __future__ import annotations

import logging
import time
from pathlib import Path

from cam_slicer.logging_config import setup_logging
from .serial_streamer import _wait_for_ok

setup_logging()
try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
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
        """Stop streaming early."""
        self._stop = True
        logging.info("Streaming stopped")

    def stream(self) -> None:
        """Send the G-code file line by line."""
        lines = [ln.strip() for ln in self.path.read_text().splitlines() if ln.strip()]
        total = len(lines)
        with serial.Serial(self.port, self.baud, timeout=1) as ser:
            logging.info(
                "Streaming %s to %s at %d baud", self.path, self.port, self.baud
            )
            idx = 0
            while idx < total and not self._stop:
                if self._paused:
                    time.sleep(0.1)
                    continue
                cmd = lines[idx]
                ser.write((cmd + "\n").encode())
                logging.info("Sent: %s", cmd)
                _wait_for_ok(ser)
                idx += 1
                print(f"{idx}/{total} lines sent", end="\r")
        print()
        logging.info("Finished streaming %s", self.path)
