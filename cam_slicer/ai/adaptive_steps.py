"""Adaptive stepdown and stepover calculations."""

import logging
from typing import Tuple

from cam_slicer.logging_config import setup_logging

setup_logging()


def compute_adaptive_steps(
    stepdown: float,
    stepover: float,
    *,
    material_hardness: float = 1.0,
    geometry_factor: float = 0.5,
    tool_load: float = 0.5,
) -> Tuple[float, float]:
    """Return adjusted stepdown and stepover for given conditions.

    Parameters
    ----------
    stepdown : float
        Base Z step.
    stepover : float
        Base lateral step.
    material_hardness : float, optional
        Relative hardness of material ``>=1``. Higher values reduce steps.
    geometry_factor : float, optional
        Complexity factor ``0-1`` where 1 means very complex geometry.
    tool_load : float, optional
        Current load on the tool ``0-1``.

    Returns
    -------
    tuple of float
        ``(stepdown, stepover)`` values after adjustment.
    """

    if stepdown <= 0 or stepover <= 0:
        raise ValueError("stepdown and stepover must be positive")

    hardness_scale = max(0.2, 1.0 / (1.0 + material_hardness))
    load_scale = max(0.2, 1.0 - 0.5 * tool_load)
    geo_scale = max(0.2, 1.0 - 0.5 * geometry_factor)
    factor = hardness_scale * load_scale * geo_scale
    new_stepdown = max(0.1, stepdown * factor)
    new_stepover = max(0.1, stepover * factor)
    logging.info(
        "Adaptive steps: down %.3f -> %.3f, over %.3f -> %.3f", stepdown, new_stepdown, stepover, new_stepover
    )
    return new_stepdown, new_stepover
