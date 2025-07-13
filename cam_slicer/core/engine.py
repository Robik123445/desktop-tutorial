"""Core toolpath utilities."""

import logging
from cam_slicer.logging_config import setup_logging

setup_logging()


def process_toolpath(toolpath, cutting_strategy: str = "conventional", radius: float = 1.0):
    """Modify a toolpath according to the milling strategy.

    Parameters
    ----------
    toolpath : sequence of tuple
        Raw ``[(x, y, z), ...]`` coordinates.
    cutting_strategy : str, optional
        ``"conventional"`` (default), ``"climb"`` or ``"adaptive"``.
    radius : float, optional
        Used by adaptive strategy to scale depth.

    Returns
    -------
    list
        Processed toolpath.

    Examples
    --------
    >>> process_toolpath([(0,0,0),(1,0,0)], 'climb')
    [(1, 0, 0), (0, 0, 0)]
    """
    strategy = cutting_strategy.lower()
    if strategy == "climb":
        logging.info("Applying climb strategy")
        return list(reversed(toolpath))
    if strategy == "adaptive":
        logging.info("Adaptive strategy radius %.2f", radius)
        factor = max(0.1, min(1.0, radius / 10))
        return [(x, y, z * factor) for x, y, z in toolpath]
    logging.info("Conventional strategy")
    return toolpath
