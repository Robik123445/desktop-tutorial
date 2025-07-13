"""Hybrid workspace scanning utilities."""

from __future__ import annotations

import logging
from typing import Callable, Iterable, Tuple, List, Dict

from cam_slicer.logging_config import setup_logging
from cam_slicer.utils.geofence import GeoFence, get_active_geofence
from cam_slicer.utils.zmap import ZMap

setup_logging()

ScanResult = Dict[str, object]


def update_global_geofence(scan_data: ScanResult) -> GeoFence:
    """Replace the active geofence using scan results."""

    fence = get_active_geofence()
    fence.forbidden_zones.clear()
    fence.air_move_zones.clear()
    for box in scan_data.get("debris") or []:
        fence.add_ai_zones_from_yolo([{"bbox": box}])
    logging.info(
        "Global geofence updated with %d zones",
        len(scan_data.get("debris") or []),
    )
    return fence

def scan_workspace(
    mode: str = "hybrid",
    image_path: str | int = "0",
    x_range: Tuple[float, float] = (0.0, 100.0),
    y_range: Tuple[float, float] = (0.0, 100.0),
    step: float = 10.0,
    probe_func: Callable[[float, float], float] | None = None,
) -> ScanResult:
    """Scan workspace using camera, probe or both.

    Parameters
    ----------
    mode : str, optional
        ``"camera"``, ``"probe"`` or ``"hybrid"``.
    image_path : str or int, optional
        Camera image path or ``0`` for webcam.
    x_range, y_range : tuple of float, optional
        Probing area for heightmap generation.
    step : float, optional
        Step distance in millimetres for probing.
    probe_func : callable, optional
        Function returning Z at given ``(x, y)``. Required for probe modes.

    Returns
    -------
    dict
        ``{"debris": list, "heightmap": ZMap | None}``
    """

    result: ScanResult = {"debris": [], "heightmap": None}

    if mode in {"camera", "hybrid"}:
        try:
            from cam_slicer.ai.debris import detect_debris  # lazy import
            result["debris"] = detect_debris(str(image_path))
            logging.info("Camera scan found %d zones", len(result["debris"]))
        except Exception as exc:  # pragma: no cover - optional dependencies
            logging.error("Camera scan failed: %s", exc)

    if mode in {"probe", "hybrid"}:
        if probe_func is None:
            raise ValueError("probe_func required for probe scan")
        from cam_slicer.sensors.probe_generator import generate_heightmap  # lazy import
        hm_dict = generate_heightmap(x_range, y_range, step, probe_func)
        points = [(x, y, z) for (x, y), z in hm_dict.items()]
        result["heightmap"] = ZMap(points)
        logging.info("Probe scan collected %d points", len(points))

    update_global_geofence(result)
    return result


def apply_scan_to_toolpath(
    toolpath: Iterable[Tuple[float, float, float]],
    scan_data: ScanResult,
    safe_z: float = 5.0,
) -> List[Tuple[float, float, float]]:
    """Adjust toolpath using scan data.

    Obstacle zones create a :class:`GeoFence` to filter points and raise Z.
    The heightmap offsets all Z values for precise surface following.
    """
    path = list(toolpath)
    fence = update_global_geofence(scan_data)
    filtered = fence.filter_toolpath(path)
    adjusted = fence.adjust_toolpath_for_air_moves(filtered, safe_z=safe_z)
    zmap = scan_data.get("heightmap")
    if isinstance(zmap, ZMap):
        adjusted = [
            (x, y, z + zmap.get_offset(x, y)) for x, y, z in adjusted
        ]
    logging.info(
        "Toolpath adjusted using %d zones and %s heightmap",
        len(scan_data.get("debris") or []),
        "with" if isinstance(zmap, ZMap) else "without",
    )
    return adjusted
