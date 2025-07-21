import logging
from typing import List, Tuple, Optional

logger = logging.getLogger(__name__)
try:
    import cv2  # type: ignore
    import numpy as np
    import trimesh  # type: ignore
except Exception as exc:  # pragma: no cover - optional dependency
    logger.warning("Optional deps missing: %s", exc)
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
