"""G-code streaming specifically for GRBL controllers."""

import logging
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
