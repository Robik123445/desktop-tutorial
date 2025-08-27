"""Command-line helpers for CAM Slicer."""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Optional

# Basic logger writing to logs/log.txt
LOG_PATH = Path("logs/log.txt")
LOG_PATH.parent.mkdir(exist_ok=True)
logger = logging.getLogger(__name__)
if not logger.handlers:
    handler = logging.FileHandler(LOG_PATH)
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)


def _print(msg: str) -> None:
    """Print helper that flushes immediately."""
    print(msg, flush=True)


def list_ports_cli() -> int:
    """List available serial ports using pyserial."""
    try:
        from serial.tools import list_ports
    except Exception:  # pragma: no cover - serial optional
        _print("pyserial nie je k dispozícii")
        logger.error("pyserial not available for listing ports")
        sys.exit(1)
    ports = [p.device for p in list_ports.comports()]
    for p in ports:
        print(p)
    return 0


def send_gcode_cli() -> int:
    """Send a G-code file to GRBL over a serial connection."""
    from cam_slicer.sender.serial_streamer import stream_gcode_to_grbl
    import serial

    ap = argparse.ArgumentParser(prog="camslicer-send", description="Send G-code to GRBL")
    ap.add_argument("gcode", help="path to .gcode file")
    ap.add_argument("port", help="/dev/ttyUSB0 etc.")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()
    gpath = Path(args.gcode)
    if not gpath.exists():
        msg = f"File not found: {gpath}"
        _print(msg)
        logger.error(msg)
        sys.exit(2)

    # optional unlock
    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            ser.write(b"\r\n\r\n")
            time.sleep(2)
            ser.reset_input_buffer()
            ser.write(b"$X\n")
            time.sleep(0.2)
    except Exception:
        logger.debug("GRBL unlock skipped", exc_info=True)

    stream_gcode_to_grbl(str(gpath), args.port, baud=args.baud)
    _print("DONE")
    logger.info("G-code %s sent to %s", gpath, args.port)
    return 0


def run_api() -> None:
    """Start uvicorn server for ``cam_slicer.api_server``."""
    import uvicorn

    uvicorn.run("cam_slicer.api_server:create_app", host="0.0.0.0", port=8000, reload=False)

