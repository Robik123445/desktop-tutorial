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
try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
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


def stream_with_recovery(
    filepath: str,
    port: str,
    baud: int = 115200,
    checkpoint: str = "last_block.json",
    resume: bool = False,
    state_reader: Optional[Callable[[], Dict[str, Any]]] = None,
) -> None:
    """Stream G-code with continuous checkpointing."""
    if serial is None:
        raise ImportError("pyserial is required for streaming")

    lines = [ln.strip() for ln in Path(filepath).read_text().splitlines() if ln.strip()]
    start = 0
    if resume:
        data = load_checkpoint(checkpoint)
        if data:
            start = data.get("line", 0) + 1
            logging.info("Resuming from line %d", start)

    with serial.Serial(port, baud, timeout=1) as ser:
        for idx, cmd in enumerate(lines):
            if idx < start:
                continue
            ser.write((cmd + "\n").encode())
            logging.info("Sent: %s", cmd)
            _wait_for_ok(ser)
            save_checkpoint(checkpoint, idx, state_reader() if state_reader else None)


def resume_job(
    filepath: str,
    port: str,
    baud: int = 115200,
    checkpoint: str = "last_block.json",
    safety_check: Optional[Callable[[], None]] = None,
    state_reader: Optional[Callable[[], Dict[str, Any]]] = None,
) -> None:
    """Resume a previously interrupted job."""
    if safety_check:
        safety_check()
    stream_with_recovery(filepath, port, baud, checkpoint, resume=True, state_reader=state_reader)
