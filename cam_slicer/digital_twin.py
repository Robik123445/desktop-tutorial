"""Live digital twin for CNC machines."""

from __future__ import annotations

import logging
import threading
import time
import json
from pathlib import Path
from typing import Callable, List, Tuple, Optional, Dict, Any

from cam_slicer.utils import ZMap
from cam_slicer.robotics.safety import validate_toolpath, DEFAULT_AXIS_RANGE

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
        key = token[0].upper()
        val = token[1:]
        if not val:
            continue
        try:
            num = float(val)
        except ValueError:
            continue
        if key == "X":
            status["x"] = num
        elif key == "Y":
            status["y"] = num
        elif key == "Z":
            status["z"] = num
        elif key == "F":
            status["feed"] = num
        elif key == "T":
            status["tool"] = int(num)
        elif key == "A":
            status["alarm"] = int(num)
    return status

class DigitalTwin:
    """Monitor machine state and provide basic simulation."""

    def __init__(self, connection: Any | None) -> None:
        """Create a new digital twin.

        Parameters
        ----------
        connection : object or None
            Object with ``readline`` method used for live monitoring. When
            ``None`` the instance can still simulate toolpaths.
        """

        self.connection = connection
        self.state: Dict[str, Any] = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.history: List[Dict[str, Any]] = []
        self._listeners: List[Callable[[Dict[str, Any]], None]] = []
        self._thread: threading.Thread | None = None
        self._running = False
        self.logger = logging.getLogger(__name__)

    def get_live_state(self) -> Dict[str, Any]:
        """Return a copy of the latest machine status."""
        return self.state.copy()

    # ------------------------------------------------------------------
    # Monitoring utilities

    def add_listener(self, cb: Callable[[Dict[str, Any]], None]) -> None:
        """Register callback invoked with every state update."""
        self._listeners.append(cb)

    def remove_listener(self, cb: Callable[[Dict[str, Any]], None]) -> None:
        """Remove a previously registered callback."""
        if cb in self._listeners:
            self._listeners.remove(cb)

    def _monitor(self) -> None:
        """Internal thread function reading lines from ``connection``."""
        while self._running:
            if not self.connection:
                time.sleep(0.01)
                continue
            try:
                line = self.connection.readline()
            except Exception as exc:  # pragma: no cover - runtime errors
                self.logger.error("Read error: %s", exc)
                time.sleep(0.1)
                continue
            if not line:
                time.sleep(0.01)
                continue
            if isinstance(line, bytes):
                line = line.decode(errors="ignore")
            status = _parse_status(str(line))
            if status:
                self.state.update(status)
                self.history.append(self.state.copy())
                for cb in list(self._listeners):
                    try:
                        cb(self.state)
                    except Exception as exc:  # pragma: no cover - log only
                        self.logger.error("Listener failed: %s", exc)

    def start_monitoring(self) -> None:
        """Begin reading status lines in a background thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._monitor, daemon=True)
        self._thread.start()
        self.logger.info("Monitoring started")

    def stop_monitoring(self) -> None:
        """Stop background monitoring."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        self.logger.info("Monitoring stopped")

    def simulate_toolpath(
        self,
        toolpath: List[Tuple[float, float, float]],
        interval: float = 0.01,
        *,
        heightmap: ZMap | None = None,
        show_3d: bool = False,
        axis_range: dict[str, tuple[float, float]] | None = None,
        max_feedrate: float | None = None,
        max_acceleration: float | None = None,
    ) -> "matplotlib.figure.Figure | list | None":
        """Simulate execution of a toolpath and optionally preview it.

        Parameters
        ----------
        toolpath : list of tuple
            Points ``[(x, y, z), ...]`` to simulate.
        interval : float, optional
            Delay in seconds between points.
        heightmap : :class:`ZMap`, optional
            Apply Z offsets from this map when provided.
        show_3d : bool, optional
            When ``True`` display a 3D preview, otherwise a 2D plot.
        axis_range : dict, optional
            Allowed coordinate range per axis.
        """
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        except Exception as exc:  # pragma: no cover - optional dep
            self.logger.warning("matplotlib not available: %s", exc)
            return toolpath

        preview_points = []
        for p in toolpath:
            x, y, z = p
            if heightmap:
                z += heightmap.get_offset(x, y)
            preview_points.append((x, y, z))
            time.sleep(interval)

        xs = [p[0] for p in preview_points]
        ys = [p[1] for p in preview_points]
        zs = [p[2] for p in preview_points]

        if show_3d:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection="3d")
            ax.plot(xs, ys, zs, color="gray", alpha=0.7)
            sc = ax.scatter(xs, ys, zs, c=zs, cmap="viridis")
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            fig.colorbar(sc, ax=ax, label="Z")
        else:
            fig, ax = plt.subplots()
            ax.plot(xs, ys, color="gray", alpha=0.7)
            sc = ax.scatter(xs, ys, c=zs, cmap="viridis")
            ax.set_aspect("equal", adjustable="box")
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            fig.colorbar(sc, ax=ax, label="Z")

        return fig

    def log_status(self, path: str | Path) -> None:
        """Write captured states to ``path`` as JSON lines."""
        data = [self.state] if not self.history else self.history
        with Path(path).open("w", encoding="utf-8") as fh:
            for entry in data:
                fh.write(json.dumps(entry) + "\n")
        self.logger.info("Logged %d states to %s", len(data), path)

    def compare_with_planned(
        self, planned: List[Tuple[float, float, float]], *, tolerance: float = 0.1
    ) -> List[int]:
        """Return indices of points deviating from ``planned`` by ``tolerance``."""
        deviations: List[int] = []
        for idx, target in enumerate(planned):
            if idx >= len(self.history):
                deviations.append(idx)
                continue
            actual = self.history[idx]
            if any(
                abs(actual.get(ax) - tgt) > tolerance
                for ax, tgt in zip(("x", "y", "z"), target)
            ):
                deviations.append(idx)
        return deviations


class WorkshopTwin:
    """Manage multiple :class:`DigitalTwin` instances."""

    def __init__(self) -> None:
        self.twins: Dict[str, DigitalTwin] = {}
        self._listeners: List[Callable[[str, Dict[str, Any]], None]] = []

    def add_machine(self, name: str, connection: Any | None) -> None:
        twin = DigitalTwin(connection)
        twin.add_listener(lambda state, n=name: self._notify(n, state))
        self.twins[name] = twin

    def _notify(self, name: str, state: Dict[str, Any]) -> None:
        for cb in list(self._listeners):
            try:
                cb(name, state)
            except Exception as exc:  # pragma: no cover - log only
                logging.error("Workshop listener failed: %s", exc)

    def add_listener(self, cb: Callable[[str, Dict[str, Any]], None]) -> None:
        self._listeners.append(cb)

    def start_all(self) -> None:
        for twin in self.twins.values():
            twin.start_monitoring()

    def stop_all(self) -> None:
        for twin in self.twins.values():
            twin.stop_monitoring()

    def get_states(self) -> Dict[str, Dict[str, Any]]:
        return {name: twin.get_live_state() for name, twin in self.twins.items()}
