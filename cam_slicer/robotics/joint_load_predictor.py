"""Estimate joint loads for a robotic arm.

This lightweight predictor computes a naive load estimate for each joint
based on current joint angles and speeds. The goal is to flag potential
overload situations without requiring a full dynamics model or external ML
framework.
"""
from __future__ import annotations

import logging
from typing import Sequence, List

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)

class JointLoadPredictor:
    """Predict approximate load on each joint using a simple heuristic."""

    def __init__(self, weight_factors: Sequence[float] | None = None) -> None:
        """Initialize predictor.

        Parameters
        ----------
        weight_factors : sequence of float, optional
            Multiplicative factors applied to each joint speed to obtain
            the load estimate. If not provided, ones are used.
        """
        self.weight_factors = list(weight_factors) if weight_factors else []

    def predict_load(
        self, joints: Sequence[float], speeds: Sequence[float]
    ) -> List[float]:
        """Return estimated load for every joint.

        The current implementation multiplies absolute joint speeds by
        optional weight factors and scales the result by the absolute joint
        angle. This provides a basic approximation of torque demand.
        """
        if len(joints) != len(speeds):
            raise ValueError("joints and speeds must be the same length")
        if self.weight_factors and len(self.weight_factors) != len(joints):
            raise ValueError("weight_factors length mismatch")
        loads = []
        for i, (angle, speed) in enumerate(zip(joints, speeds)):
            factor = self.weight_factors[i] if i < len(self.weight_factors) else 1.0
            load = abs(angle) * abs(speed) * factor
            loads.append(load)
        logger.debug("Predicted loads: %s", loads)
        return loads
