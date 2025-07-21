"""Object detection helpers using YOLOv8."""
from __future__ import annotations

import logging
from pathlib import Path

from cam_slicer.logging_config import setup_logging
from cam_slicer.robotics.safety import emergency_stop
from cam_slicer.utils.geofence import GeoFence, get_active_geofence

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

    results = model(image)
    boxes: list[tuple[float, float, float, float]] = []
    for res in results:
        for box in res.boxes.xyxy.tolist():
            boxes.append(tuple(float(v) for v in box))
    logger.info("Detected %d objects", len(boxes))
    return boxes

def run_live_detection(camera_index: int = 0, pixel_to_mm: float = 0.1) -> None:
    """Display live camera detections and update the global ``GeoFence``.

    Parameters
    ----------
    camera_index : int, optional
        Index of the camera to open. Defaults to ``0``.
    pixel_to_mm : float, optional
        Conversion factor from pixels to millimetres. Defaults to ``0.1``.
    """

    if YOLO is None or cv2 is None:  # pragma: no cover - optional deps
        raise ImportError("ultralytics and opencv-python are required")

    model = YOLO("yolov8n.pt")
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {camera_index}")

    fence = get_active_geofence()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            results = model(frame)
            for res in results:
                names = getattr(res, "names", {})
                for box, cls_idx in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist()):
                    label = names.get(int(cls_idx), str(int(cls_idx)))
                    x1, y1, x2, y2 = [float(v) for v in box]
                    w_mm = (x2 - x1) * pixel_to_mm
                    h_mm = (y2 - y1) * pixel_to_mm

                    danger = label in {"person", "hand"}
                    if label in {"debris", "tool", "clamp", "person", "hand"}:
                        fence.add_ai_zones_from_yolo([{"bbox": (x1, y1, x2, y2)}])

                    color = (0, 0, 255) if danger else (0, 255, 0)
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    cv2.putText(
                        frame,
                        label,
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        2,
                    )
                    cv2.putText(
                        frame,
                        f"{w_mm:.1f}x{h_mm:.1f} mm",
                        (int(x1), int(y2) + 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        2,
                    )

                    if danger:
                        print(f"WARNING: {label} detected")
                        emergency_stop()

            cv2.imshow("CAM", frame)
            if cv2.waitKey(1) == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
