"""Interactive G-code streaming for GRBL controllers."""

from __future__ import annotations

import argparse
import logging
import threading
import time
from pathlib import Path
from typing import Optional

from cam_slicer.logging_config import setup_logging
setup_logging()
logger = logging.getLogger(__name__)
try:
    import serial  # type: ignore
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
