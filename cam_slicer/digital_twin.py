"""Live digital twin for CNC machines."""

from __future__ import annotations

import logging
import threading
import time
import json
from typing import Callable, List, Tuple, Optional, Dict, Any



from cam_slicer.logging_config import setup_logging
setup_logging()
def _parse_status(line: str) -> Dict[str, Any]:
    """Parse a status line from the controller.

    Parameters
    ----------
    line : str
        Raw status string such as ``"X0 Y0 Z0 F100 T1 A0"``.

    Returns
    -------
    dict
        Parsed values with keys ``x``, ``y``, ``z``, ``feed``, ``tool`` and
        ``alarm`` when present.

    Examples
    --------
    >>> _parse_status("X1 Y2 Z0 F500")
    {'x': 1.0, 'y': 2.0, 'z': 0.0, 'feed': 500.0}
    """
    status: Dict[str, Any] = {}
    for token in line.strip().split():
        if token.startswith("X"):
            status["x"] = float(token[1:])
        elif token.startswith("Y"):
            status["y"] = float(token[1:])
        elif token.startswith("Z"):
            status["z"] = float(token[1:])
        elif token.startswith("F"):
            status["feed"] = float(token[1:])
        elif token.startswith("T"):
            status["tool"] = token[1:]
        elif token.startswith("A"):
            status["alarm"] = token[1:]
    return status


class DigitalTwin:
    """Digital twin for real-time CNC telemetry.

    Parameters
    ----------
    connection : object
        Object with ``readline`` or ``recv`` method returning controller status
        lines.

    Examples
    --------
    >>> twin = DigitalTwin(serial_connection)  # doctest: +SKIP
    >>> twin.start_monitoring()               # doctest: +SKIP
    """

    def __init__(self, connection: Any) -> None:
        self.connection = connection
        self.state: Dict[str, Any] = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "feed": 0.0,
            "tool": None,
            "alarm": None,
        }
        self.history: List[Dict[str, Any]] = []
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._listeners: List[Callable[[Dict[str, Any]], None]] = []
        logging.info("DigitalTwin initialized")

    def start_monitoring(self) -> None:
        """Start a background thread reading controller status.

        The method spawns a daemon thread which continuously polls the
        connection for status lines until :meth:`stop_monitoring` is called.
        """

        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logging.info("DigitalTwin monitoring started")

    def stop_monitoring(self) -> None:
        """Stop the monitoring thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            logging.info("DigitalTwin monitoring stopped")

    def _read_line(self) -> Optional[str]:
        """Read one status line from the connection.

        Returns ``None`` on communication errors or when the connection does not
        provide a read method.
        """
        try:
            if hasattr(self.connection, "readline"):
                data = self.connection.readline()
                if isinstance(data, bytes):
                    data = data.decode()
                return data
            if hasattr(self.connection, "recv"):
                data = self.connection.recv(1024)
                if isinstance(data, bytes):
                    data = data.decode()
                return data
            return None
        except Exception as exc:  # pragma: no cover - log only
            logging.error("Reading connection failed: %s", exc)
            return None

    def _run(self) -> None:
        while self._running:
            line = self._read_line()
            if not line:
                time.sleep(0.01)
                continue
            status = _parse_status(line)
            if status:
                self.state.update(status)
                self.history.append(self.state.copy())
                for cb in list(self._listeners):
                    try:
                        cb(self.state)
                    except Exception as exc:  # pragma: no cover - log only
                        logging.error("Listener failed: %s", exc)
        
    def add_listener(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """Register callback invoked with each state update."""
        self._listeners.append(callback)

    def log_status(self, filepath: str = "twin_report.txt") -> None:
        """Append the current state to ``filepath`` in JSON format."""
        with open(filepath, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(self.state) + "\n")
        logging.info("DigitalTwin status logged to %s", filepath)

    def compare_with_planned(
        self, toolpath: List[Tuple[float, float, float]], tolerance: float = 0.5
    ) -> List[Tuple[int, float, float, float]]:
        """Compare recorded positions with a reference toolpath.

        Parameters
        ----------
        toolpath : list of tuple
            Planned points ``[(x, y, z), ...]``.
        tolerance : float, optional
            Maximum allowed deviation before reporting.

        Returns
        -------
        list of tuple
            Deviations as ``(index, dx, dy, dz)``.
        """
        deviations: List[Tuple[int, float, float, float]] = []
        for idx, real in enumerate(self.history[: len(toolpath)]):
            planned = toolpath[idx]
            dx = real.get("x", 0) - planned[0]
            dy = real.get("y", 0) - planned[1]
            dz = real.get("z", 0) - planned[2]
            if abs(dx) > tolerance or abs(dy) > tolerance or abs(dz) > tolerance:
                deviations.append((idx, dx, dy, dz))
                logging.warning(
                    "Deviation at %d: dx=%.3f dy=%.3f dz=%.3f", idx, dx, dy, dz
                )
        return deviations

    def get_live_state(self) -> Dict[str, Any]:
        """Return a copy of the latest machine status."""
        return self.state.copy()
    def simulate_toolpath(self, toolpath: List[Tuple[float, float, float]], interval: float = 0.01) -> None:
        """Simulate execution of a toolpath without hardware.

        Parameters
        ----------
        toolpath : list of tuple
            Points ``[(x, y, z), ...]`` to simulate.
        interval : float, optional
            Delay in seconds between points.
        """
        for pt in toolpath:
            self.state.update({"x": pt[0], "y": pt[1], "z": pt[2]})
            self.history.append(self.state.copy())
            for cb in list(self._listeners):
                try:
                    cb(self.state)
                except Exception as exc:  # pragma: no cover - log only
                    logging.error("Listener failed: %s", exc)
            time.sleep(interval)

    def replay_history(self, history: List[Dict[str, Any]], interval: float = 0.01) -> None:
        """Replay a previously recorded history."""
        for state in history:
            self.state.update(state)
            for cb in list(self._listeners):
                try:
                    cb(self.state)
                except Exception as exc:  # pragma: no cover - log only
                    logging.error("Listener failed: %s", exc)
            time.sleep(interval)

class WorkshopTwin:
    """Aggregate multiple :class:`DigitalTwin` instances."""

    def __init__(self) -> None:
        self.twins: Dict[str, DigitalTwin] = {}
        self._listeners: List[Callable[[str, Dict[str, Any]], None]] = []

    def add_machine(self, name: str, connection: Any) -> DigitalTwin:
        """Create and store a twin for a machine."""
        twin = DigitalTwin(connection)
        self.twins[name] = twin
        twin.add_listener(lambda state, n=name: self._notify(n, state))
        return twin

    def _notify(self, name: str, state: Dict[str, Any]) -> None:
        for cb in list(self._listeners):
            try:
                cb(name, state)
            except Exception as exc:  # pragma: no cover - log only
                logging.error("Listener failed: %s", exc)

    def add_listener(self, callback: Callable[[str, Dict[str, Any]], None]) -> None:
        """Register callback receiving ``(machine_name, state)``."""
        self._listeners.append(callback)

    def start_all(self) -> None:
        """Start monitoring on all twins."""
        for twin in self.twins.values():
            twin.start_monitoring()

    def stop_all(self) -> None:
        """Stop monitoring on all twins."""
        for twin in self.twins.values():
            twin.stop_monitoring()

    def get_states(self) -> Dict[str, Dict[str, Any]]:
        """Return live state for each machine."""
        return {name: twin.get_live_state() for name, twin in self.twins.items()}

    def simulate(self, toolpaths: Dict[str, List[Tuple[float, float, float]]], interval: float = 0.01) -> None:
        """Run simulation for all machines using provided toolpaths."""
        for name, tp in toolpaths.items():
            if name in self.twins:
                self.twins[name].simulate_toolpath(tp, interval)

    def replay(self, histories: Dict[str, List[Dict[str, Any]]], interval: float = 0.01) -> None:
        """Replay recorded histories for all machines."""
        for name, hist in histories.items():
            if name in self.twins:
                self.twins[name].replay_history(hist, interval)

