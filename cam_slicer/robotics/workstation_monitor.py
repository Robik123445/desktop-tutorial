"""Live diagnostics for multiple robotic workstations."""
from __future__ import annotations

import logging
import time
from typing import Callable, Dict

from cam_slicer.logging_config import setup_logging
from cam_slicer.digital_twin import WorkshopTwin
from .joint_load_predictor import JointLoadPredictor

setup_logging()
logger = logging.getLogger(__name__)


def start_live_diagnostics(
    twin: WorkshopTwin,
    predictor: JointLoadPredictor | None = None,
    interval: float = 0.5,
) -> None:
    """Periodically log load estimates for all machines."""
    if predictor is None:
        predictor = JointLoadPredictor()

    def _on_update(name: str, state: Dict[str, float]) -> None:
        joints = state.get("joints")
        speeds = state.get("speeds")
        if isinstance(joints, list) and isinstance(speeds, list):
            loads = predictor.predict_load(joints, speeds)
            logger.info("%s loads: %s", name, loads)

    twin.add_listener(_on_update)
    twin.start_all()
    try:
        while True:
            time.sleep(interval)
    except KeyboardInterrupt:
        pass
    finally:
        twin.stop_all()

