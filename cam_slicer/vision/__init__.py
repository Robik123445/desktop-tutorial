"""Camera-based detection utilities."""

try:  # optional dependencies may be missing (e.g., cv2, ultralytics)
    from .debris_detector import detect_objects, run_live_detection
    from .board_detector import (
        detect_board_position,
        get_transform_from_detection,
        auto_transform_gcode,
        process_camera_to_gcode,
    )
except Exception:  # pragma: no cover - optional
    detect_objects = run_live_detection = None
    detect_board_position = get_transform_from_detection = None
    auto_transform_gcode = process_camera_to_gcode = None

__all__ = [
    "detect_objects",
    "run_live_detection",
    "detect_board_position",
    "get_transform_from_detection",
    "auto_transform_gcode",
    "process_camera_to_gcode",
]
