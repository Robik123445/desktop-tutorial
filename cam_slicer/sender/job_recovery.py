import json
import logging
import time
from pathlib import Path
from typing import Any, Callable, Dict, Optional

try:
    import serial
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


def stream_with_recovery(
    gcode_path: str,
    port: str,
    *,
    baud: int = 115200,
    checkpoint: str | Path,
    state_cb: Optional[Callable[[str], Dict[str, Any]]] = None,
) -> None:
    """Stream G-code while storing progress to ``checkpoint``."""

    if serial is None:
        raise ImportError("pyserial is required for streaming")

    path = Path(gcode_path)
    if not path.is_file():
        raise FileNotFoundError(path)

    ckpt_path = Path(checkpoint)
    with serial.Serial(port, baud, timeout=1) as ser:
        for idx, raw in enumerate(path.read_text().splitlines()):
            line = raw.strip()
            if not line:
                continue
            ser.write((line + "\n").encode())
            _wait_for_ok(ser)
            save_checkpoint(ckpt_path, idx, state_cb(line) if state_cb else None)


def resume_job(
    gcode_path: str,
    port: str,
    *,
    baud: int = 115200,
    checkpoint: str | Path,
) -> None:
    """Resume streaming from the line after the checkpoint."""

    if serial is None:
        raise ImportError("pyserial is required for streaming")

    data = load_checkpoint(checkpoint)
    start = (data["line"] + 1) if data else 0

    path = Path(gcode_path)
    if not path.is_file():
        raise FileNotFoundError(path)

    lines = path.read_text().splitlines()
    if start >= len(lines):
        return

    ckpt_path = Path(checkpoint)
    with serial.Serial(port, baud, timeout=1) as ser:
        for idx, raw in enumerate(lines[start:], start=start):
            line = raw.strip()
            if not line:
                continue
            ser.write((line + "\n").encode())
            _wait_for_ok(ser)
            save_checkpoint(ckpt_path, idx)
