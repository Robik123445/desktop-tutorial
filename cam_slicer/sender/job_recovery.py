from __future__ import annotations

"""Job recovery utilities for reliable CNC streaming."""

import json
import logging
import time
from pathlib import Path
from typing import Callable, Dict, Any, Optional

from cam_slicer.logging_config import setup_logging
from .serial_streamer import _wait_for_ok

setup_logging()
logger = logging.getLogger(__name__)
try:
    import serial  # type: ignore
except ModuleNotFoundError as exc:  # pragma: no cover - optional dependency
    logger.warning("pyserial not installed: %s", exc)
    serial = None  # type: ignore


def save_checkpoint(path: str | Path, line: int, state: Optional[Dict[str, Any]] = None) -> None:
    """Write progress info to ``path`` as JSON."""
    data = {
        "line": line,
        "timestamp": time.time(),
        "state": state or {},
    }
    Path(path).write_text(json.dumps(data))
    logging.info("Checkpoint saved at line %d", line)


def load_checkpoint(path: str | Path) -> Optional[Dict[str, Any]]:
    """Return checkpoint data or ``None`` if unavailable."""
    p = Path(path)
    if not p.is_file():
        return None
    try:
        return json.loads(p.read_text())
    except Exception as exc:  # pragma: no cover - log only
        logging.error("Failed to load checkpoint: %s", exc)
        return None
