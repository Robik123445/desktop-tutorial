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
except ModuleNotFoundError as exc:  # pragma: no cover - optional
    logger.warning("ultralytics not installed: %s", exc)
    YOLO = None  # type: ignore

try:
    import cv2  # type: ignore
except ModuleNotFoundError as exc:  # pragma: no cover - optional
    logger.warning("opencv-python not installed: %s", exc)
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
