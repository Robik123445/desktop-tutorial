"""Diagnostic helpers for robotic arm mode."""
from __future__ import annotations

import logging
from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


def log_run(profile: str, line_count: int) -> None:
    """Log start of a robotic run."""
    logger.info("Robot run started profile=%s lines=%d", profile, line_count)


def log_warning(message: str) -> None:
    """Log a warning related to robot operation."""
    logger.warning("Robot warning: %s", message)


def log_error(message: str) -> None:
    """Log an error during robotic operation."""
    logger.error("Robot error: %s", message)


def log_feedback(text: str) -> None:
    """Log user feedback or bug report."""
    logger.info("Robot feedback: %s", text)
