"""Object detection helpers using YOLOv8."""
from __future__ import annotations

import logging
from pathlib import Path

from cam_slicer.logging_config import setup_logging
from cam_slicer.robotics.safety import emergency_stop
from cam_slicer.utils.geofence import get_active_geofence, GeoFence

setup_logging()
logger = logging.getLogger(__name__)

try:  # optional dependencies
    from ultralytics import YOLO
except ModuleNotFoundError as exc:  # pragma: no cover - optional
    logger.warning("ultralytics not installed: %s", exc)
    YOLO = None  # type: ignore

try:
    import cv2  # type: ignore
except ModuleNotFoundError as exc:  # pragma: no cover - optional
    logger.warning("opencv-python not installed: %s", exc)
    cv2 = None  # type: ignore


def detect_objects(image_path: str) -> list[tuple[float, float, float, float]]:
    """Detect generic objects in an image using YOLOv8."""
    if YOLO is None or cv2 is None:
        raise ImportError("ultralytics and opencv-python are required")

    model = YOLO("yolov8n.pt")

    if image_path == "0":
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()
        if not ret:
            raise RuntimeError("Failed to capture frame from camera")
        image = frame
    else:
        img_path = Path(image_path)
        if not img_path.is_file():
            raise FileNotFoundError(img_path)
        image = cv2.imread(str(img_path))
        if image is None:
            raise RuntimeError(f"Failed to read image: {img_path}")
