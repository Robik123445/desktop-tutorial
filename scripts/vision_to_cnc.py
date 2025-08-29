"""Compatibility wrapper for the old vision_to_cnc script."""

import argparse
import logging

try:  # pragma: no cover - cv2 may be missing on CI
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore

import cam_slicer.vision_bridge as _vb
from cam_slicer.vision import camera_config as cc

M_affine = _vb.M_affine


def px_to_xy(px):
    _vb.M_affine = M_affine
    return _vb.px_to_xy(px)


def write_move_gcode(x, y, path="move_to_target.gcode"):
    _vb.M_affine = M_affine
    return _vb.write_move_gcode(x, y, path=path)


run_main = _vb.main

__all__ = ["M_affine", "px_to_xy", "write_move_gcode"]

LOG_PATH = "logs/log.txt"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    handlers=[logging.FileHandler(LOG_PATH), logging.StreamHandler()],
)
logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    """Parsuje CLI argumenty."""
    p = argparse.ArgumentParser(description="Vision->CNC helper")
    p.add_argument("--cam", type=int, default=None, help="Index kamery")
    return p.parse_args()


def select_index(args: argparse.Namespace) -> int:
    """Zvolí index kamery: CLI > config > 0."""
    idx = args.cam if args.cam is not None else cc.get_current_index(default=0)
    logger.info("Používam kameru index=%d (CLI má prednosť pred configom)", idx)
    return idx


def open_camera(idx: int) -> "cv2.VideoCapture":
    """Otvára kameru a overí dostupnosť."""
    if cv2 is None:
        logger.error("OpenCV nie je dostupné")
        raise SystemExit(1)
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        logger.error("Kameru %d sa nepodarilo otvoriť", idx)
        raise SystemExit(1)
    cc.set_current_index(idx)
    return cap


def main() -> None:
    """Spustí interaktívny proces."""
    args = parse_args()
    idx = select_index(args)
    cap = open_camera(idx)
    cap.release()
    _vb.M_affine = M_affine
    run_main()


if __name__ == "__main__":
    main()
