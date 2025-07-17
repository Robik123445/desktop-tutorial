"""Live digital twin for CNC machines."""

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
        # ... (zvyšok funkcie ostáva nezmenený)
        pass

class DigitalTwin:
    # ... __init__ a ostatné metódy ...

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
        max_feedrate : float, optional
            Raise ``ValueError`` if exceeded.
        max_acceleration : float, optional
            Acceleration limit for safety checks.

        Returns
        -------
        matplotlib.figure.Figure or list or None
            The preview figure when matplotlib is available. If not,
            the processed point list is returned instead.
        """

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

    # ... zvyšok triedy ...
