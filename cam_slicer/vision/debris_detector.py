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
except ModuleNotFoundError:  # pragma: no cover - optional
    YOLO = None  # type: ignore

try:
    import cv2  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional
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

    results = model(image)
    boxes: list[tuple[float, float, float, float]] = []
    for res in results:
        for box in res.boxes.xyxy.tolist():
            tup = tuple(float(v) for v in box)
            boxes.append(tup)
            logger.info("Detected box: %s", tup)
    return boxes


DETECTION_CLASSES = {"debris", "tool", "clamp", "hand", "person"}


def run_live_detection(geofence: GeoFence | None = None) -> None:
    """Stream webcam feed and update geofence when obstacles appear."""
    if YOLO is None or cv2 is None:
        raise ImportError("ultralytics and opencv-python are required")

    model = YOLO("yolov8n.pt")
    if geofence is None:
        try:
            geofence = get_active_geofence()
        except Exception:
            geofence = None
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            results = model(frame)
            for res in results:
                names = getattr(res, "names", {})
                for box, cls_idx in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist()):
                    x1, y1, x2, y2 = (int(v) for v in box)
                    label = names.get(int(cls_idx), str(int(cls_idx)))
                    warn = label in {"person", "hand"}
                    color = (0, 0, 255) if warn else (0, 255, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    width_mm = (x2 - x1) * 0.1
                    height_mm = (y2 - y1) * 0.1
                    dim_text = f"{width_mm:.1f}x{height_mm:.1f} mm"
                    cv2.putText(frame, dim_text, (x1, y2 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    logger.info("Live box %s label %s dims %s", (x1, y1, x2, y2), label, dim_text)
                    if label in DETECTION_CLASSES and geofence is not None:
                        geofence.add_ai_zones_from_yolo([
                            {"label": label, "bbox": (x1 * 0.1, y1 * 0.1, x2 * 0.1, y2 * 0.1)}
                        ])
                    if warn:
                        logger.warning("WARNING: %s detected", label)
                        print(f"WARNING: {label} detected")
                        try:
                            emergency_stop()
                        except Exception as exc:  # pragma: no cover - best effort
                            logger.error("Emergency stop failed: %s", exc)
            cv2.imshow("live", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        if hasattr(cv2, "destroyAllWindows"):
            cv2.destroyAllWindows()
