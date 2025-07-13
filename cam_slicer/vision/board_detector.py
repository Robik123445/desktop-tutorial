"""Board detection and toolpath alignment helpers."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import List

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)

try:  # optional deps
    from ultralytics import YOLO
except ModuleNotFoundError:  # pragma: no cover - optional
    YOLO = None  # type: ignore

try:
    import cv2  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional
    cv2 = None  # type: ignore


from cam_slicer.robotics.kinematics import TransformConfig
from cam_slicer.core.header_footer import ControllerConfig


def detect_board_position(image_path: str) -> dict:
    """Detect a board in an image and return its centre and size."""
    if YOLO is None or cv2 is None:
        raise ImportError("ultralytics and opencv-python are required")

    model = YOLO("yolov8n.pt")
    img_path = Path(image_path)
    if not img_path.is_file():
        raise FileNotFoundError(img_path)
    image = cv2.imread(str(img_path))
    if image is None:
        raise RuntimeError(f"Failed to read image: {img_path}")

    results = model(image)
    for res in results:
        names = getattr(res, "names", {})
        for box, cls_idx in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist()):
            label = names.get(int(cls_idx), str(int(cls_idx)))
            if label == "board":
                x1, y1, x2, y2 = (float(v) for v in box)
                info = {
                    "x": (x1 + x2) / 2.0,
                    "y": (y1 + y2) / 2.0,
                    "width": x2 - x1,
                    "height": y2 - y1,
                }
                logger.info("Board position: %s", info)
                return info
    logger.info("Board not detected")
    return {}


def get_transform_from_detection(detection: dict) -> TransformConfig:
    """Create a TransformConfig from detection data."""
    px_to_mm = 0.1
    offset_x = float(detection.get("x", 0.0)) * px_to_mm
    offset_y = float(detection.get("y", 0.0)) * px_to_mm
    cfg = TransformConfig(offset=(offset_x, offset_y, 0.0))
    logger.info("Created transform from detection: %s", cfg)
    return cfg


def auto_transform_gcode(toolpath: List[tuple], detection: dict) -> List[str]:
    """Generate G-code for a toolpath aligned to a detected board."""
    if not toolpath:
        return []

    transform = get_transform_from_detection(detection)

    if "width" in detection and "height" in detection:
        px_to_mm = 0.1
        xs = [p[0] for p in toolpath]
        ys = [p[1] for p in toolpath]
        tool_w = max(max(xs) - min(xs), 1e-6)
        tool_h = max(max(ys) - min(ys), 1e-6)
        board_w = float(detection["width"]) * px_to_mm
        board_h = float(detection["height"]) * px_to_mm
        scale = min(board_w / tool_w, board_h / tool_h)
        transform.scale = scale
        cx = (min(xs) + max(xs)) / 2 * scale
        cy = (min(ys) + max(ys)) / 2 * scale
        board_cx = float(detection.get("x", 0.0)) * px_to_mm
        board_cy = float(detection.get("y", 0.0)) * px_to_mm
        transform.offset = (board_cx - cx, board_cy - cy, transform.offset[2])

    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    logger.info("Auto transforming toolpath with detection %s", detection)
    from cam_slicer.core.gcode_export import toolpath_to_gcode
    return toolpath_to_gcode(toolpath, cfg, transform)


def process_camera_to_gcode(toolpath: List[tuple], image_path: str, output_file: str = "auto_output.gcode") -> bool:
    """Full workflow: detect board and export aligned G-code."""
    try:
        info = detect_board_position(image_path)
        if not info:
            logger.info("No board detected in %s", image_path)
            return False
        gcode = auto_transform_gcode(toolpath, info)
        Path(output_file).write_text("\n".join(gcode), encoding="utf-8")
        logger.info("G-code saved to %s", output_file)
        return True
    except Exception as exc:  # pragma: no cover - log failure
        logger.error("process_camera_to_gcode failed: %s", exc)
        return False
