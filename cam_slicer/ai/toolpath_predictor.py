import logging
import math
from collections import defaultdict
from typing import List, Tuple, Dict, Sequence

from cam_slicer.logging_config import setup_logging
from cam_slicer.robotics.joint_load_predictor import JointLoadPredictor

setup_logging()
logger = logging.getLogger(__name__)


class ToolpathPredictor:
    """Estimate machining time, tool wear and load per operation."""

    def __init__(self, wear_factor: float = 0.001) -> None:
        """Initialize predictor.

        Parameters
        ----------
        wear_factor : float, optional
            Constant multiplied by machining time to estimate tool wear.
        """
        self.wear_factor = wear_factor

    def predict(
        self,
        toolpath: Sequence[Tuple[float, float, float]],
        feedrate: float,
        operations: Sequence[str] | None = None,
    ) -> Dict[str, object]:
        """Return machining time, wear and load per operation.

        Parameters
        ----------
        toolpath : sequence of tuple
            XYZ points describing the toolpath.
        feedrate : float
            Linear feedrate in mm/min.
        operations : sequence of str, optional
            Operation name for each segment (length ``len(toolpath) - 1``).

        Returns
        -------
        dict
            ``{"time_sec": float, "tool_wear": float, "load_per_operation": {..}}``
        """
        if len(toolpath) < 2:
            return {"time_sec": 0.0, "tool_wear": 0.0, "load_per_operation": {}}

        op_lengths: Dict[str, float] = defaultdict(float)
        for i in range(1, len(toolpath)):
            p1 = toolpath[i - 1]
            p2 = toolpath[i]
            seg_len = math.sqrt(
                (p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2
            )
            op = operations[i - 1] if operations and i - 1 < len(operations) else "default"
            op_lengths[op] += seg_len

        total_len = sum(op_lengths.values())
        time_sec = total_len / (feedrate / 60.0)
        tool_wear = time_sec * self.wear_factor

        # Rough load estimate proportional to segment length
        load = {op: length for op, length in op_lengths.items()}
        logger.debug("Predicted time %.2fs wear %.4f", time_sec, tool_wear)
        return {"time_sec": time_sec, "tool_wear": tool_wear, "load_per_operation": load}
