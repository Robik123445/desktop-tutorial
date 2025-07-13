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
try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
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

    def is_paused(self) -> bool:
        return self._paused

    def should_stop(self) -> bool:
        return self._stop


def _input_listener(ctrl: StreamController) -> None:
    """Background thread listening for user commands."""
    print("Press 'p' to pause/resume, 'q' to quit")
    while not ctrl.should_stop():
        try:
            cmd = input().strip().lower()
        except EOFError:  # pragma: no cover - no stdin
            break
        if cmd == "p":
            if ctrl.is_paused():
                ctrl.resume()
                print("Resumed")
            else:
                ctrl.pause()
                print("Paused")
        elif cmd == "q":
            ctrl.stop()
            break


def stream_gcode_interactive(
    filepath: str,
    port: str,
    baud: int = 115200,
    controller: Optional[StreamController] = None,
) -> None:
    """Stream ``filepath`` to a GRBL controller with pause/resume support."""

    if serial is None:
        raise ImportError("pyserial is required for streaming")

    path = Path(filepath)
    if not path.is_file():
        raise FileNotFoundError(path)

    lines = [ln.strip() for ln in path.read_text().splitlines() if ln.strip()]
    total = len(lines)
    ctrl = controller or StreamController()

    with serial.Serial(port, baud, timeout=1) as ser:
        logging.info("Streaming %s to %s at %d baud", filepath, port, baud)
        idx = 0
        while idx < total and not ctrl.should_stop():
            if ctrl.is_paused():
                time.sleep(0.1)
                continue

            cmd = lines[idx]
            ser.write((cmd + "\n").encode())
            logging.info("Sent: %s", cmd)
            _wait_for_ok(ser)
            idx += 1
            print(f"{idx}/{total} lines sent", end="\r")

    print()
    logging.info("Finished streaming %s", filepath)


def main(argv: Optional[list[str]] = None) -> None:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description="Stream G-code to GRBL")
    parser.add_argument("gcode", help="Path to G-code file")
    parser.add_argument("port", help="Serial port device")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args(argv)

    ctrl = StreamController()
    thread = threading.Thread(target=_input_listener, args=(ctrl,), daemon=True)
    thread.start()

    try:
        stream_gcode_interactive(args.gcode, args.port, args.baud, ctrl)
    finally:
        ctrl.stop()
        thread.join()


if __name__ == "__main__":  # pragma: no cover - manual execution
    main()
