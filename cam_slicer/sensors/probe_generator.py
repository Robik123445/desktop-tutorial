import json
import logging
from pathlib import Path
from typing import Callable, Tuple, Dict

from cam_slicer.logging_config import setup_logging
setup_logging()
# Configure logging to common file

def generate_heightmap(
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    step: float | None,
    probe_func: Callable[[float, float], float],
    density: int | None = None,
) -> Dict[Tuple[float, float], float]:
    """Probe a rectangular grid and return a heightmap.

    Parameters
    ----------
    x_range, y_range : tuple of float
        Start and end coordinates for probing.
    step : float, optional
        Fixed step between probe points. Ignored when ``density`` is provided.
    probe_func : Callable[[float, float], float]
        Function returning the ``z`` height at a given ``(x, y)``.
    density : int, optional
        Number of samples along each axis. ``step`` is derived from this value.

    Returns
    -------
    dict
        Mapping ``(x, y)`` to measured ``z`` heights.

    Examples
    --------
    >>> hm = generate_heightmap((0,1), (0,1), 0.5, lambda x, y: 0.0)
    >>> list(hm.items())[:1]
    [((0, 0), 0.0)]
    """

    if density is not None:
        if density < 2:
            raise ValueError("density must be >= 2")
        x_span = x_range[1] - x_range[0]
        y_span = y_range[1] - y_range[0]
        step_x = x_span / (density - 1)
        step_y = y_span / (density - 1)
    elif step is not None and step > 0:
        step_x = step_y = step
    else:
        raise ValueError("step or density must be provided and positive")

    x_start, x_end = x_range
    y_start, y_end = y_range
    heightmap: Dict[Tuple[float, float], float] = {}

    x = x_start
    while x <= x_end + 1e-9:
        y = y_start
        while y <= y_end + 1e-9:
            z = probe_func(x, y)
            heightmap[(x, y)] = z
            logging.info("Probed (%.3f, %.3f) -> %.3f", x, y, z)
            y += step_y
        x += step_x

    return heightmap


def export_heightmap_to_json(heightmap: Dict[Tuple[float, float], float], filepath: str | Path) -> None:
    """Export a heightmap to a JSON file.

    Parameters
    ----------
    heightmap : dict
        Mapping ``(x, y)`` to ``z`` values as produced by
        :func:`generate_heightmap`.
    filepath : str or Path
        Output path for the JSON file.

    Examples
    --------
    >>> hm = {(0,0): 0.0}
    >>> export_heightmap_to_json(hm, 'hm.json')  # doctest: +SKIP
    """

    path = Path(filepath)
    data = {"points": [{"x": x, "y": y, "z": z} for (x, y), z in heightmap.items()]}
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")
    logging.info("Saved heightmap to %s", filepath)
