"""Camera-based detection utilities."""

from .debris_detector import detect_objects, run_live_detection
from .board_detector import (
    detect_board_position,
    get_transform_from_detection,
    auto_transform_gcode,
    process_camera_to_gcode,
)

__all__ = [
    "detect_objects",
    "run_live_detection",
    "detect_board_position",
    "get_transform_from_detection",
    "auto_transform_gcode",
    "process_camera_to_gcode",
]
