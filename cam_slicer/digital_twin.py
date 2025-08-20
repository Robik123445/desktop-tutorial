"""Lightweight digital twin utilities with deterministic behaviour."""
from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Dict, Iterable, List, Tuple

from .robotics.safety import validate_toolpath

# configure module level logger writing to ``log.txt``
_logger = logging.getLogger(__name__)
_handler = logging.FileHandler("log.txt")
_handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
_logger.addHandler(_handler)
_logger.setLevel(logging.INFO)
_logger.propagate = False

# shared buffer of processed preview points
preview_points: List[Tuple[float, float, float]] = []


class DigitalTwin:
    """Simple state model of a single CNC machine."""

    def __init__(self, name: str | None) -> None:
        """Create a new twin with optional ``name``."""
        self.name = name or "machine"
        self.state: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 0.0, "feed": 0.0, "speed": 0.0}
        self._listeners: List[Callable[[Dict[str, float]], None]] = []
        _logger.info("DigitalTwin %s initialised", self.name)

    # public API -----------------------------------------------------------
    def add_listener(self, cb: Callable[[Dict[str, float]], None]) -> None:
        """Register ``cb`` to be called on state updates."""
        self._listeners.append(cb)

    def get_live_state(self) -> Dict[str, float]:
        """Return a copy of the current machine state."""
        return dict(self.state)

    def simulate_toolpath(
        self,
        toolpath: Iterable[Tuple[float, float, float]],
        *,
        interval: float = 0.0,
        delay: bool = True,
        axis_range: Dict[str, Tuple[float, float]] | None = None,
        heightmap: object | None = None,
        max_feedrate: float | None = None,
        max_acceleration: float | None = None,
    ) -> List[str]:
        """Simulate ``toolpath`` and populate ``preview_points``.

        Parameters
        ----------
        toolpath:
            Iterable of XYZ tuples.
        interval:
            Sleep duration between points when ``delay`` is ``True``.
        delay:
            If ``False``, no sleeping occurs.
        axis_range, max_feedrate, max_acceleration:
            Forwarded to :func:`validate_toolpath`.
        heightmap:
            Optional object providing ``get_offset(x, y)``.

        Returns
        -------
        list[str]
            Validation warnings produced by :func:`validate_toolpath`.
        """
        points = list(toolpath)
        warnings = validate_toolpath(
            points,
            axis_range=axis_range,
            max_feedrate=max_feedrate,
            max_acceleration=max_acceleration,
        )
        preview_points.clear()
        for x, y, z in points:
            if heightmap is not None:
                z += float(heightmap.get_offset(x, y))
            self.state["x"], self.state["y"], self.state["z"] = float(x), float(y), float(z)
            preview_points.append((self.state["x"], self.state["y"], self.state["z"]))
            self._notify()
            if delay and interval > 0:
                time.sleep(interval)
        _logger.info("%s simulated %d points", self.name, len(preview_points))
        return warnings

    # internal helpers -----------------------------------------------------
    def _notify(self) -> None:
        for cb in self._listeners:
            cb(self.get_live_state())


class WorkshopTwin:
    """Manage a group of :class:`DigitalTwin` instances."""

    def __init__(self) -> None:
        """Initialise empty workshop."""
        self.twins: Dict[str, DigitalTwin] = {}
        self._conns: Dict[str, object] = {}
        self._threads: Dict[str, threading.Thread] = {}
        self._stops: Dict[str, threading.Event] = {}
        self._listeners: List[Callable[[str, Dict[str, float]], None]] = []
        _logger.info("WorkshopTwin initialised")

    def add_listener(self, cb: Callable[[str, Dict[str, float]], None]) -> None:
        """Register callback for any machine update."""
        self._listeners.append(cb)

    def _forward(self, name: str, state: Dict[str, float]) -> None:
        for cb in self._listeners:
            cb(name, state)

    def add_machine(self, name: str, connection: object | None) -> DigitalTwin:
        """Add a machine with optional streaming ``connection``."""
        twin = DigitalTwin(name)
        self.twins[name] = twin
        self._conns[name] = connection
        twin.add_listener(lambda s, n=name: self._forward(n, s))
        return twin

    def start_all(self) -> None:
        """Start threads reading from machine connections."""
        for name, conn in self._conns.items():
            if conn is None or name in self._threads:
                continue
            stop = threading.Event()
            thread = threading.Thread(target=self._reader, args=(name, conn, stop), daemon=True)
            self._threads[name] = thread
            self._stops[name] = stop
            thread.start()
        _logger.info("WorkshopTwin threads started")

    def _reader(self, name: str, conn: object, stop: threading.Event) -> None:
        twin = self.twins[name]
        while not stop.is_set():
            line = conn.readline()
            if not line:
                continue
            parts = line.strip().split()
            for part in parts:
                axis = part[0].lower()
                try:
                    val = float(part[1:])
                except ValueError:
                    continue
                if axis in ("x", "y", "z"):
                    twin.state[axis] = val
            twin._notify()
        _logger.info("Reader for %s stopped", name)

    def stop_all(self) -> None:
        """Stop all machine threads."""
        for name, stop in list(self._stops.items()):
            stop.set()
        for name, thread in list(self._threads.items()):
            thread.join(timeout=0.1)
        self._threads.clear()
        self._stops.clear()
        _logger.info("WorkshopTwin threads stopped")

    def get_states(self) -> Dict[str, Dict[str, float]]:
        """Return live states for all machines."""
        return {name: twin.get_live_state() for name, twin in self.twins.items()}
