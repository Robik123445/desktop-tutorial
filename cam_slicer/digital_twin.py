"""Live digital twin for CNC machines."""

from __future__ import annotations

import logging
import threading
import time
import json
from typing import Callable, List, Tuple, Optional, Dict, Any

from cam_slicer.utils import ZMap
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
            status["tool"] = int(token[1:])
        elif token.startswith("A"):
            status["alarm"] = int(token[1:])
    return status


class DigitalTwin:
    """Digital twin object for live CNC feedback."""

    def __init__(self, connection: Any):
        self.connection = connection
        self.state: Dict[str, Any] = {}
        self.history: List[Dict[str, Any]] = []
        self._listeners: List[Callable[[Dict[str, Any]], None]] = []
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def add_listener(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        self._listeners.append(callback)

    def remove_listener(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        self._listeners.remove(callback)

    def start(self, poll_interval: float = 0.05) -> None:
        if self._running:
            return
        self._running = True

        def loop():
            while self._running:
                try:
                    line = self.connection.readline()
                    status = _parse_status(line)
                    self.state.update(status)
                    self.history.append(self.state.copy())
                    for cb in list(self._listeners):
                        cb(self.state)
                except Exception as exc:
                    logging.error("Twin poll failed: %s", exc)
                time.sleep(poll_interval)

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)
            self._thread = None

    def export_history(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(self.history, f, indent=2)

    def compare_with_toolpath(self, toolpath: List[Tuple[float, float, float]], tolerance: float = 0.1) -> List[Tuple[int, float, float, float]]:
        """Compare machine history to planned toolpath.

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

    def simulate_toolpath(
        self,
        toolpath: List[Tuple[float, float, float]],
        interval: float = 0.01,
        *,
        heightmap: ZMap | None = None,
        show_3d: bool = False,
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

        Returns
        -------
        matplotlib.figure.Figure or list or None
            The preview figure when matplotlib is available. If not,
            the processed point list is returned instead.
        """

        preview_points: List[Tuple[float, float, float]] = []
        for pt in toolpath:
            x, y, z = pt
            if heightmap is not None:
                z += heightmap.get_offset(x, y)
            self.state.update({"x": x, "y": y, "z": z})
            self.history.append(self.state.copy())
            preview_points.append((x, y, z))
            for cb in list(self._listeners):
                try:
                    cb(self.state)
                except Exception as exc:  # pragma: no cover - log only
                    logging.error("Listener failed: %s", exc)
            time.sleep(interval)

        logging.info("Simulated %d toolpath points", len(toolpath))

        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        except Exception as exc:  # pragma: no cover - if matplotlib missing
            logging.error("Matplotlib not available: %s", exc)
            return preview_points

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
            except Exception as exc:
                logging.error("Workshop listener failed: %s", exc)

    def add_listener(self, callback: Callable[[str, Dict[str, Any]], None]) -> None:
        self._listeners.append(callback)
