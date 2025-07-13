"""Debris detection and avoidance utilities."""

from pathlib import Path
from typing import Iterable, List, Tuple
import logging

from cam_slicer.logging_config import setup_logging
from cam_slicer.utils.geofence import GeoFence

setup_logging()

try:  # optional dependencies
    from ultralytics import YOLO
except ModuleNotFoundError:  # pragma: no cover - optional
    YOLO = None  # type: ignore

try:
    import cv2  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional
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
        ret, frame = cap.read()
        cap.release()
        if not ret:
            raise RuntimeError("Failed to capture frame")
        image = frame
    else:
        img_path = Path(image_path)
        if not img_path.is_file():
            raise FileNotFoundError(img_path)
        image = cv2.imread(str(img_path))
        if image is None:
            raise RuntimeError(f"Failed to read image: {img_path}")

    results = model(image)
    boxes: List[BoundingBox] = []
    for res in results:
        names = getattr(res, "names", {})
        for box, cls_idx in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist()):
            label = names.get(int(cls_idx), str(int(cls_idx)))
            if label in {"debris", "obstacle"}:
                tup = tuple(float(v) for v in box)
                boxes.append(tup)
                logging.info("Debris detected %s at %s", label, tup)
    return boxes


def add_debris_zones_from_image(
    image_path: str,
    fence: GeoFence,
    zone_type: str = "forbidden",
    reminder_threshold: int = 5,
) -> bool:
    """Detect debris from an image and update ``GeoFence`` zones.

    Parameters
    ----------
    image_path : str
        Path to source image or ``"0"`` for webcam.
    fence : :class:`GeoFence`
        Fence instance to update.
    zone_type : str, optional
        ``"forbidden"`` or ``"air_move"`` for resulting zones.
    reminder_threshold : int, optional
        If number of detections >= threshold, return ``True`` to trigger a
        cleaning reminder.

    Returns
    -------
    bool
        ``True`` if user should be reminded to clean the table.
    """
    boxes = detect_debris(image_path)
    detections = [{"label": "debris", "bbox": b} for b in boxes]
    fence.add_ai_zones_from_yolo(detections, zone_type=zone_type)
    logging.info("Added %d debris zones", len(boxes))
    return len(boxes) >= reminder_threshold


def plan_toolpath_avoiding_debris(
    toolpath: List[Tuple[float, float, float]],
    fence: GeoFence,
    safe_z: float = 5.0,
) -> List[Tuple[float, float, float]]:
    """Filter and adjust a toolpath to avoid debris zones."""
    filtered = fence.filter_toolpath(toolpath)
    return fence.adjust_toolpath_for_air_moves(filtered, safe_z=safe_z)
