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
except Exception as exc:  # pragma: no cover - optional
    logger.warning("ultralytics not available: %s", exc)
    YOLO = None  # type: ignore

try:
    import cv2  # type: ignore
except Exception as exc:  # pragma: no cover - optional
    logger.warning("opencv-python not available: %s", exc)
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
        success, frame = cap.read()
        cap.release()
        if not success:
            logger.error("Failed to capture frame from webcam")
            return []
        image = frame
    else:
        image = str(image_path)

    results = model(image)
    boxes: List[BoundingBox] = []
    for res in results:  # pragma: no cover - heavy computation
        for x1, y1, x2, y2 in res.boxes.xyxy.tolist():
            boxes.append((float(x1), float(y1), float(x2), float(y2)))
    logger.info("Detected %d debris regions", len(boxes))
    return boxes


def add_debris_zones_from_image(image: str | Path, fence: GeoFence) -> List[BoundingBox]:
    """Update ``fence`` with debris zones detected in ``image``.

    The function relies on :func:`detect_debris` which in turn depends on
    optional imaging libraries. When those libraries are missing the function
    simply logs the issue and returns an empty list, allowing callers to proceed
    without special handling.

    Parameters
    ----------
    image : str or Path
        Path to the image file to analyze. ``"0"`` activates a webcam capture.
    fence : GeoFence
        Geofence instance that will receive new forbidden zones.

    Returns
    -------
    list of tuple
        List of added bounding boxes. Empty if detection could not run.
    """

    logger.info("Scanning %s for debris", image)
    try:
        boxes = detect_debris(str(image))
    except ImportError as exc:  # pragma: no cover - optional
        logger.warning("Debris detection skipped: %s", exc)
        return []

    if not boxes:
        logger.info("No debris found in %s", image)
        return []

    for x1, y1, x2, y2 in boxes:
        fence.forbidden_zones.append((x1, y1, x2, y2))
    logger.info("Added %d debris zones to geofence", len(boxes))
    return boxes


def plan_toolpath_avoiding_debris(
    toolpath: Iterable[Tuple[float, float, float]],
    fence: GeoFence,
    safe_z: float = 5.0,
) -> List[Tuple[float, float, float]]:
    """Filter and adjust ``toolpath`` based on debris zones.

    The toolpath is first filtered using :meth:`GeoFence.filter_toolpath` and
    then raised to ``safe_z`` inside air-move zones via
    :meth:`GeoFence.adjust_toolpath_for_air_moves`.
    """

    original = list(toolpath)
    logger.info("Planning path with %d points", len(original))
    filtered = fence.filter_toolpath(original)
    adjusted = fence.adjust_toolpath_for_air_moves(filtered, safe_z=safe_z)
    logger.info("Path reduced to %d points after debris avoidance", len(adjusted))
    return adjusted
