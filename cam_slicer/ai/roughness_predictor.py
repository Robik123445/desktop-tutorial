import logging

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


def predict_surface_roughness(feedrate: float, tool_diameter: float, rpm: float, *, material_factor: float = 1.0) -> float:
    """Estimate surface roughness (Ra) in micrometers.

    Parameters
    ----------
    feedrate : float
        Linear feedrate in mm/min.
    tool_diameter : float
        Tool diameter in mm.
    rpm : float
        Spindle speed in revolutions per minute.
    material_factor : float, optional
        Multiplier reflecting material hardness or brittleness.

    Returns
    -------
    float
        Estimated average surface roughness Ra in micrometers.
    """
    if rpm <= 0 or tool_diameter <= 0:
        raise ValueError("rpm and tool_diameter must be positive")

    feed_per_rev = feedrate / rpm
    roughness = material_factor * (feed_per_rev ** 2) / (8 * tool_diameter) * 1000
    logger.debug("Predicted roughness %.3f um", roughness)
    return roughness
