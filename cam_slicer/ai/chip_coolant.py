import logging
from typing import Sequence, Tuple, List, Dict

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


class ChipCoolantAdvisor:
    """Suggest chip evacuation moves and coolant settings."""

    MATERIAL_COOLANT = {
        "aluminum": "air blast",
        "steel": "flood",
        "plywood": "vacuum",
        "mdf": "vacuum",
        "acrylic": "air blast",
    }

    def __init__(self, material: str = "aluminum", safe_height: float = 5.0, segment_threshold: float = 50.0) -> None:
        self.material = material.lower()
        self.safe_height = safe_height
        self.segment_threshold = segment_threshold

    def analyze(self, toolpath: Sequence[Tuple[float, float, float]], feedrate: float) -> Dict[str, List[Tuple[float, float, float]]]:
        """Analyze toolpath and return chip evacuation moves and coolant type."""
        if len(toolpath) < 2:
            raise ValueError("toolpath must contain at least two points")

        coolant = self.MATERIAL_COOLANT.get(self.material, "air blast")
        evac_moves: List[Tuple[float, float, float]] = []
        acc_dist = 0.0
        last = toolpath[0]
        for pt in toolpath[1:]:
            dx = pt[0] - last[0]
            dy = pt[1] - last[1]
            dist = (dx * dx + dy * dy) ** 0.5
            acc_dist += dist
            if acc_dist >= self.segment_threshold:
                evac_moves.append((pt[0], pt[1], self.safe_height))
                acc_dist = 0.0
            last = pt
        logger.info("Coolant '%s', %d evacuation moves", coolant, len(evac_moves))
        return {"coolant": coolant, "evac_moves": evac_moves}


def apply_chip_evacuations(toolpath: Sequence[Tuple[float, float, float]], evac_moves: Sequence[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
    """Insert evacuation moves at the end of the toolpath."""
    result = list(toolpath)
    result.extend(evac_moves)
    logger.debug("Inserted %d evacuation moves", len(evac_moves))
    return result
