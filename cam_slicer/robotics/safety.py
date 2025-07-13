"""Safety helpers for robotic workflows."""
import logging
from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


def emergency_stop() -> None:
    """Trigger an emergency stop and log the action."""
    logger.warning("Emergency stop activated")
    print("EMERGENCY STOP triggered!")
