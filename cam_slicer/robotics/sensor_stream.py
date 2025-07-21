import threading
import logging
import asyncio
from typing import Callable, Optional

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


class SensorStream:
    """Read live pose data from serial or websocket.

    Parameters
    ----------
    source : str
        Serial port name or websocket URL.
    mode : str, optional
        ``"serial"`` or ``"websocket"``. Defaults to ``"serial"``.
    baud : int, optional
        Serial baud rate when ``mode`` is serial. Defaults to 115200.
    callback : Callable[[tuple], None], optional
        Called with ``(x, y, z, a, b, c)`` for each received frame.
    """

    def __init__(
        self,
        source: str,
        *,
        mode: str = "serial",
        baud: int = 115200,
        callback: Optional[Callable[[tuple], None]] = None,
    ) -> None:
        self.source = source
        self.mode = mode
        self.baud = baud
        self.callback = callback
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.last_pose: Optional[tuple] = None

    def start(self) -> None:
        """Start background reading thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info("SensorStream started mode=%s source=%s", self.mode, self.source)

    def stop(self) -> None:
        """Stop reading thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
        logger.info("SensorStream stopped")

    # Internal helpers -----------------------------------------------------
    def _run(self) -> None:
        if self.mode == "websocket":
            asyncio.run(self._run_ws())
        else:
            self._run_serial()

    def _run_serial(self) -> None:
        try:
            with serial.Serial(self.source, self.baud, timeout=1) as ser:
                while self._running:
                    line = ser.readline().decode().strip()
                    if line:
                        self._handle_line(line)
        except Exception as exc:  # pragma: no cover - serial port errors
            logger.error("Serial error: %s", exc)

    async def _run_ws(self) -> None:
        import websockets

        try:
            async with websockets.connect(self.source) as ws:
                while self._running:
                    line = await ws.recv()
                    if line:
                        self._handle_line(line)
        except Exception as exc:  # pragma: no cover - connection errors
            logger.error("WebSocket error: %s", exc)

    def _handle_line(self, line: str) -> None:
        try:
            parts = [float(x) for x in line.replace(",", " ").split()]
            if len(parts) >= 6:
                pose = tuple(parts[:6])
                self.last_pose = pose
                if self.callback:
                    self.callback(pose)
        except ValueError:
            logger.debug("Ignoring invalid line: %s", line)
