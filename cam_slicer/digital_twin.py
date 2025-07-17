from __future__ import annotations
import logging
import threading
import time
import json
from typing import Callable, List, Tuple, Optional, Dict, Any

from cam_slicer.utils import ZMap
from cam_slicer.robotics.safety import validate_toolpath, DEFAULT_AXIS_RANGE
from cam_slicer.logging_config import setup_logging
setup_logging()

def _parse_status(line: str) -> Dict[str, Any]:
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
            status["a"] = float(token[1:])
        elif token.startswith("ALARM"):
            status["alarm"] = token
    return status

class DigitalTwin:
    def __init__(self, connection: Any):
        self.connection = connection
        self.state: Dict[str, Any] = {}
        self.history: List[Dict[str, Any]] = []
        self._listeners: List[Callable[[Dict[str, Any]], None]] = []
        self._monitor_thread: Optional[threading.Thread] = None
        self._running = False

    def add_listener(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        self._listeners.append(callback)

    def remove_listener(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        if callback in self._listeners:
            self._listeners.remove(callback)

    def start_monitoring(self) -> None:
        if self._monitor_thread and self._monitor_thread.is_alive():
            return
        self._running = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()

    def stop_monitoring(self) -> None:
        self._running = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=1)

    def _monitor_loop(self) -> None:
        while self._running:
            try:
                line = self.connection.readline()
                if not line:
                    time.sleep(0.01)
                    continue
                state = _parse_status(line)
                if state:
                    self.state.update(state)
                    self.history.append(self.state.copy())
                    for cb in list(self._listeners):
                        try:
                            cb(self.state)
                        except Exception as exc:
                            logging.error("Listener failed: %s", exc)
            except Exception as exc:
                logging.error("Monitor loop error: %s", exc)

    def log_status(self, path: str) -> None:
        with open(path, "w") as f:
            for state in self.history:
                f.write(json.dumps(state) + "\n")

    def compare_with_planned(self, toolpath: List[Tuple[float, float, float]], tolerance: float = 0.02) -> List[Tuple[int, float, float, float]]:
        deviations: List[Tuple[int, float, float, float]] = []
        for idx, real in enumerate(self.history[: len(toolpath)]):
            planned = toolpath[idx]
            dx = real.get("x", 0) - planned[0]
            dy = real.get("y", 0) - planned[1]
            dz = real.get("z", 0) - planned[2]
            if abs(dx) > tolerance or abs(dy) > tolerance or abs(dz) > tolerance:
                deviations.append((idx, dx, dy, dz))
                logging.warning("Deviation at %d: dx=%.3f dy=%.3f dz=%.3f", idx, dx, dy, dz)
        return deviations

    def get_live_state(self) -> Dict[str, Any]:
        return self.state.copy()

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
        warnings = validate_toolpath(
            toolpath,
            axis_range=axis_range or DEFAULT_AXIS_RANGE,
            max_feedrate=max_feedrate,
            max_acceleration=max_acceleration,
        )
        if warnings:
            logging.warning("Simulation warnings: %s", "; ".join(warnings))

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
                except Exception as exc:
                    logging.error("Listener failed: %s", exc)
            time.sleep(interval)

        logging.info("Simulated %d toolpath points", len(toolpath))

        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        except Exception as exc:
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
        for state in history:
            self.state.update(state)
            for cb in list(self._listeners):
                try:
                    cb(self.state)
                except Exception as exc:
                    logging.error("Listener failed: %s", exc)
            time.sleep(interval)

class WorkshopTwin:
    def __init__(self) -> None:
        self.twins: Dict[str, DigitalTwin] = {}
        self._listeners: List[Callable[[str, Dict[str, Any]], None]] = []

    def add_machine(self, name: str, connection: Any) -> DigitalTwin:
        twin = DigitalTwin(connection)
        self.twins[name] = twin
        twin.add_listener(lambda state, n=name: self._notify(n, state))
        return twin

    def _notify(self, name: str, state: Dict[str, Any]) -> None:
        for cb in list(self._listeners):
            try:
                cb(name, state)
            except Exception as exc:
                logging.error("WorkshopTwin listener failed: %s", exc)

    def add_listener(self, callback: Callable[[str, Dict[str, Any]], None]) -> None:
        self._listeners.append(callback)
