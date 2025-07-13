import logging
from typing import List, Tuple, Optional

try:
    import cv2  # type: ignore
    import numpy as np
    import trimesh  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    cv2 = None
    np = None
    trimesh = None

from cam_slicer.logging_config import setup_logging

setup_logging()


def generate_surface_texture(
    mesh: 'trimesh.Trimesh',
    pattern: str = "stippling",
    spacing: float = 1.0,
    amplitude: float = 0.5,
    grayscale_map: Optional[str] = None,
) -> List[List[Tuple[float, float, float]]]:
    """Generate decorative texture toolpaths on a mesh surface.

    Parameters
    ----------
    mesh : trimesh.Trimesh
        Target mesh for the texture.
    pattern : str, optional
        One of ``"stippling"``, ``"wave"`` or ``"map"``.
    spacing : float, optional
        Approximate distance between pattern points.
    amplitude : float, optional
        Depth or height variation for the texture.
    grayscale_map : str, optional
        Path to a grayscale image used when ``pattern='map'``.

    Returns
    -------
    list of list
        Sequence of toolpath segments describing the texture.
    """
    if trimesh is None:
        raise ImportError("trimesh is required for surface texturing")
    if pattern == "map" and (cv2 is None or np is None):
        raise ImportError("opencv-python and numpy are required for map pattern")

    bounds = mesh.bounds
    (xmin, ymin, _), (xmax, ymax, zmax) = bounds
    width = xmax - xmin
    height = ymax - ymin
    toolpaths: List[List[Tuple[float, float, float]]] = []

    if pattern == "stippling":
        x = xmin
        toggle = 1
        while x <= xmax:
            y = ymin if toggle > 0 else ymin + spacing / 2
            while y <= ymax:
                pt = mesh.nearest.on_surface([[x, y, zmax]])[0]
                toolpaths.append([(pt[0], pt[1], pt[2] - amplitude)])
                y += spacing
            x += spacing
            toggle *= -1
        logging.info("Generated %d stippling points", len(toolpaths))

    elif pattern == "wave":
        steps = int(width / spacing) + 1
        for i in range(steps):
            x = xmin + i * spacing
            path: List[Tuple[float, float, float]] = []
            y = ymin
            while y <= ymax:
                z = amplitude * np.sin(2 * np.pi * (y - ymin) / height)
                pt = mesh.nearest.on_surface([[x, y, zmax]])[0]
                path.append((pt[0], pt[1], pt[2] + z))
                y += spacing
            toolpaths.append(path)
        logging.info("Generated %d wave paths", len(toolpaths))

    elif pattern == "map" and grayscale_map:
        img = cv2.imread(grayscale_map, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(grayscale_map)
        rows, cols = img.shape
        sx = width / cols
        sy = height / rows
        for r in range(rows):
            path: List[Tuple[float, float, float]] = []
            for c in range(cols):
                intensity = img[r, c] / 255.0
                x = xmin + c * sx
                y = ymin + r * sy
                z_offset = amplitude * (intensity - 0.5)
                pt = mesh.nearest.on_surface([[x, y, zmax]])[0]
                path.append((pt[0], pt[1], pt[2] + z_offset))
            toolpaths.append(path)
        logging.info("Generated texture from grayscale map %s", grayscale_map)
    else:
        raise ValueError(f"Unknown pattern: {pattern}")

    return toolpaths
