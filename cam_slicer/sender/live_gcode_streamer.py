"""Class-based G-code streamer with pause and stop controls."""

from __future__ import annotations

import logging
import time
from pathlib import Path

from cam_slicer.logging_config import setup_logging
from .serial_streamer import _wait_for_ok

setup_logging()
logger = logging.getLogger(__name__)
try:
    import serial  # type: ignore
except ModuleNotFoundError as exc:  # pragma: no cover - optional dependency
    logger.warning("pyserial not installed: %s", exc)
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
