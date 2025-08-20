from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from cam_slicer.logging_config import setup_logging
setup_logging()
logger = logging.getLogger(__name__)
_log_path = Path("logs/log.txt")
if not any(
    isinstance(h, logging.FileHandler) and getattr(h, "baseFilename", "") == str(_log_path)
    for h in logging.getLogger().handlers
):
    _log_path.parent.mkdir(exist_ok=True)
    fh = logging.FileHandler(_log_path)
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logging.getLogger().addHandler(fh)
try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    cv2 = None
try:
    import numpy as np
except Exception:  # pragma: no cover - optional dependency
    np = None
import xml.etree.ElementTree as ET


def _skeletonize(img: Any) -> Any:
    """Return morphological skeleton of a binary image."""
    if cv2 is None or np is None:
        logger.error("cv2 library not available for skeletonization")
        return img
    skel = np.zeros(img.shape, np.uint8) if np is not None else img.copy()
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while True:
        eroded = cv2.erode(img, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(img, temp)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded.copy()
        if cv2.countNonZero(img) == 0:
            break
    return skel

def import_and_vectorize_image(filepath: str, mode: str = "outline") -> list[list[tuple[float, float]]]:
    """Load PNG/JPG and vectorize to a list of paths."""
    if cv2 is None:
        logger.error("cv2 library not available")
        return []
    img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
    if img is None:
        logger.error("Failed to load image %s", filepath)
        return []
    logger.info("Loaded image %s", filepath)
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
    if mode == "centerline":
        binary = _skeletonize(binary)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    paths: list[list[tuple[float, float]]] = []
    for cnt in contours:
        pts = [(float(p[0][0]), float(p[0][1])) for p in cnt]
        if pts:
            paths.append(pts)
    logger.info("Vectorized %d paths from %s", len(paths), filepath)
    return paths

def _parse_points(attr: str) -> list[tuple[float, float]]:
    vals = [float(v) for v in attr.replace(',', ' ').split() if v]
    return list(zip(vals[::2], vals[1::2]))

def import_svg(filepath: str) -> list[list[tuple[float, float]]]:
    """Parse basic SVG shapes into paths."""
    try:
        tree = ET.parse(filepath)
        root = tree.getroot()
    except Exception as exc:  # pragma: no cover - file errors
        logger.error("Failed to read SVG %s: %s", filepath, exc)
        return []
    ns_strip = lambda tag: tag.split('}')[-1]
    paths: list[list[tuple[float, float]]] = []
    for elem in root.iter():
        tag = ns_strip(elem.tag)
        if tag in {"polyline", "polygon"} and elem.get("points"):
            pts = _parse_points(elem.get("points"))
            if pts:
                paths.append(pts)
        elif tag == "path" and elem.get("d"):
            d = elem.get("d")
            curr: list[tuple[float, float]] = []
            nums = ''
            cmd = ''
            for ch in d:
                if ch.isalpha():
                    if cmd in {'M', 'L'} and nums:
                        vals = [float(v) for v in nums.replace(',', ' ').split() if v]
                        curr.extend(list(zip(vals[::2], vals[1::2])))
                    cmd = ch.upper()
                    nums = ''
                else:
                    nums += ch
            if cmd in {'M', 'L'} and nums:
                vals = [float(v) for v in nums.replace(',', ' ').split() if v]
                curr.extend(list(zip(vals[::2], vals[1::2])))
            if curr:
                paths.append(curr)
    logger.info("Imported %d paths from %s", len(paths), filepath)
    return paths
