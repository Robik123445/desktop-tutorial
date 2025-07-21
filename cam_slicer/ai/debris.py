"""Debris detection and avoidance utilities."""

from pathlib import Path
from typing import Iterable, List, Tuple
import logging

from cam_slicer.logging_config import setup_logging
from cam_slicer.utils.geofence import GeoFence

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

BoundingBox = Tuple[float, float, float, float]


def detect_debris(image_path: str) -> List[BoundingBox]:
    """Detect debris or obstacles in an image using YOLOv8.

    Parameters
    ----------
    image_path : str
        Path to image or ``"0"`` for webcam capture.

    Returns
    -------
    list of tuple
        Bounding boxes ``(x1, y1, x2, y2)`` for detected debris.
    """
    if YOLO is None or cv2 is None:
        raise ImportError("ultralytics and opencv-python are required")

    model = YOLO("yolov8n.pt")

    if image_path == "0":
        cap = cv2.VideoCapture(0)
