"""Serial streaming utilities."""

from __future__ import annotations

import logging
from pathlib import Path

from .serial_stream import stream_gcode_live
from .serial_streamer import stream_gcode_to_grbl
from .grbl_streamer import stream_gcode_interactive
from .live_gcode_streamer import LiveGcodeStreamer
from .feedback_streamer import stream_gcode_with_feedback
from .job_recovery import (
    stream_with_recovery,
    resume_job,
    save_checkpoint,
    load_checkpoint,
)

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)
_log_path = Path("logs/log.txt")
if not any(
    isinstance(h, logging.FileHandler) and getattr(h, "baseFilename", "") == str(_log_path)
    for h in logging.getLogger().handlers
):
    _log_path.parent.mkdir(exist_ok=True)
    fh = logging.FileHandler(_log_path)
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logging.getLogger().addHandler(fh)

try:  # optional dependency
    from serial.tools import list_ports
except Exception:  # pragma: no cover - optional
    list_ports = None  # type: ignore


def list_available_ports() -> list[str]:
    """Return available serial port device names."""
    if list_ports is None:
        logger.warning("pyserial not installed; no ports available")
        return []
    return [p.device for p in list_ports.comports()]


__all__ = [
    "stream_gcode_live",
    "stream_gcode_to_grbl",
    "stream_gcode_interactive",
    "LiveGcodeStreamer",
    "stream_with_recovery",
    "resume_job",
    "save_checkpoint",
    "load_checkpoint",
    "stream_gcode_with_feedback",
    "list_available_ports",
]
