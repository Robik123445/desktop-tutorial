"""Utilities for keeping toolpaths away from forbidden zones.

This module provides the :class:`GeoFence` class used to filter or adjust
toolpaths based on manually defined or AI detected rectangular regions.
"""

import logging
from typing import Iterable, List, Tuple


from cam_slicer.logging_config import setup_logging
setup_logging()
class GeoFence:
    """Filter toolpaths using forbidden or air-move rectangular zones.

    Parameters
    ----------
    forbidden_zones : iterable of tuple, optional
        Zones ``(x1, y1, x2, y2)`` that the tool must avoid.
    air_move_zones : iterable of tuple, optional
        Zones where rapid Z moves should be inserted.
    """

    def __init__(
        self,
        forbidden_zones: Iterable[Tuple[float, float, float, float]] | None = None,
        air_move_zones: Iterable[Tuple[float, float, float, float]] | None = None,
    ) -> None:
        """Initialize the geofence."""

        self.forbidden_zones: List[Tuple[float, float, float, float]] = (
            list(forbidden_zones) if forbidden_zones else []
        )
        self.air_move_zones: List[Tuple[float, float, float, float]] = (
            list(air_move_zones) if air_move_zones else []
        )
        logging.info(
            "GeoFence initialized with %d forbidden and %d air-move zones",
            len(self.forbidden_zones),
            len(self.air_move_zones),
        )

    def is_inside(self, x: float, y: float) -> bool:
        """Return ``True`` if a point lies inside any forbidden zone."""
        for x1, y1, x2, y2 in self.forbidden_zones:
            x_low, x_high = sorted((x1, x2))
            y_low, y_high = sorted((y1, y2))
            if x_low <= x <= x_high and y_low <= y <= y_high:
                return True
        return False

    def _is_inside_air(self, x: float, y: float) -> bool:
        """Return ``True`` if point lies in an air-move zone."""
        for x1, y1, x2, y2 in self.air_move_zones:
            x_low, x_high = sorted((x1, x2))
            y_low, y_high = sorted((y1, y2))
            if x_low <= x <= x_high and y_low <= y <= y_high:
                return True
        return False

    def filter_toolpath(
        self, toolpath: List[Tuple[float, float, float]]
    ) -> List[Tuple[float, float, float]]:
        """Remove points that fall inside forbidden zones.

        Parameters
        ----------
        toolpath : list of tuple
            Original toolpath ``[(x, y, z), ...]``.

        Returns
        -------
        list of tuple
            Filtered toolpath without forbidden points.
        """
        filtered: List[Tuple[float, float, float]] = []
        for x, y, z in toolpath:
            if self.is_inside(x, y):
                logging.warning("Point inside geofence removed: X%.3f Y%.3f", x, y)
                continue
            filtered.append((x, y, z))
        logging.info(
            "Toolpath filtered: %d -> %d points", len(toolpath), len(filtered)
        )
        return filtered

    def adjust_toolpath_for_air_moves(
        self,
        toolpath: List[Tuple[float, float, float]],
        safe_z: float = 5.0,
    ) -> List[Tuple[float, float, float]]:
        """Raise ``Z`` to ``safe_z`` when passing through air-move zones."""

        adjusted: List[Tuple[float, float, float]] = []
        for x, y, z in toolpath:
            if self._is_inside_air(x, y):
                if z < safe_z:
                    logging.info(
                        "Raising Z at X%.3f Y%.3f from %.3f to %.3f", x, y, z, safe_z
                    )
                    z = safe_z
            adjusted.append((x, y, z))
        return adjusted

    def add_ai_zones_from_yolo(
        self,
        detections: Iterable[dict],
        zone_type: str = "forbidden",
    ) -> None:
        """Add rectangular zones from YOLO detections."""

        count = 0
        for det in detections:
            box = det.get("bbox")
            if not box or len(box) != 4:
                continue
            x1, y1, x2, y2 = box
            if zone_type == "air_move":
                self.air_move_zones.append((x1, y1, x2, y2))
            else:
                self.forbidden_zones.append((x1, y1, x2, y2))
            count += 1
        logging.info("Added %d %s zones from AI", count, zone_type)


# Global geofence used by scanners and live detection
ACTIVE_GEOFENCE = GeoFence()


def get_active_geofence() -> GeoFence:
    """Return the shared geofence instance."""

    return ACTIVE_GEOFENCE


def reset_active_geofence() -> None:
    """Clear zones from the shared geofence."""

    ACTIVE_GEOFENCE.forbidden_zones.clear()
    ACTIVE_GEOFENCE.air_move_zones.clear()
    logging.info("Active geofence reset")
