import logging
from typing import Optional

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


class HeadSelector:
    """Select the optimal tool head based on operation and material."""

    default_map = {
        "cut": {"wood": "laser", "plywood": "laser", "mdf": "laser", "acrylic": "laser", "default": "spindle"},
        "engrave": {"wood": "laser", "plywood": "laser", "mdf": "laser", "acrylic": "laser", "default": "spindle"},
        "mill": {"default": "spindle"},
        "print": {"default": "print_head"},
    }

    def select_head(
        self,
        operation: str,
        material: str = "unknown",
        geometry: str = "2D",
        override: Optional[str] = None,
    ) -> str:
        """Return recommended tool head.

        Parameters
        ----------
        operation : str
            Operation type such as ``cut`` or ``engrave``.
        material : str
            Material name. Used for heuristics.
        geometry : str, optional
            ``2D`` or ``3D`` geometry hint.
        override : str, optional
            If provided, this head is returned directly.
        """
        if override:
            logger.info("Manual head override: %s", override)
            return override
        op_map = self.default_map.get(operation, {"default": "spindle"})
        key = material.lower()
        head = op_map.get(key, op_map.get("default", "spindle"))
        if geometry == "3D" and head == "laser":
            head = "spindle"
        logger.debug(
            "Selected head %s for operation=%s material=%s geometry=%s",
            head,
            operation,
            material,
            geometry,
        )
        return head
