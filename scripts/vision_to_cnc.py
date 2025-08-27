#!/usr/bin/env python3
"""Interactive camera-to-CNC pipeline.

This script connects a notebook camera with a GRBL-controlled CNC.
It supports three-point affine calibration, target selection by mouse
or optional ArUco marker, conversion of pixel coordinates to machine
coordinates and safe streaming of G-code to GRBL.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Optional, Tuple

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - cv2 may be missing
    cv2 = None  # type: ignore
import numpy as np

from cam_slicer.sender.serial_streamer import stream_gcode_to_grbl

try:  # optional serial import for unlocking GRBL
    import serial
except Exception:  # pragma: no cover - serial is optional
    serial = None

# --- constants -------------------------------------------------------------
SAFE_Z = 10.0      # safe clearance height in mm
RAPID = 1500.0     # rapid feed rate in mm/min
PORT = "/dev/ttyUSB0"
BAUD = 115200
CALIB_PATH = Path("vision_affine.npz")

# optional ArUco support
try:
    aruco = cv2.aruco if cv2 is not None else None
    if aruco is not None:
        ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    else:
        ARUCO_DICT = None
except Exception:  # pragma: no cover - ArUco is optional
    aruco = None
    ARUCO_DICT = None

# --- logging setup ---------------------------------------------------------
LOG_PATH = Path("logs/log.txt")
LOG_PATH.parent.mkdir(exist_ok=True)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    handlers=[logging.FileHandler(LOG_PATH), logging.StreamHandler()]
)
logger = logging.getLogger(__name__)

# --- global state ----------------------------------------------------------
last_click: Optional[Tuple[int, int]] = None
cal_px: list[Tuple[int, int]] = []
cal_xy: list[Tuple[float, float]] = []
M_affine: Optional[np.ndarray] = None  # 2x3 matrix


# --- utility functions -----------------------------------------------------
def grbl_unlock(port: str = PORT, baud: int = BAUD) -> None:
    """Send unlock command to GRBL controller if serial is available."""
    if serial is None:
        logger.debug("Serial module not available; skipping unlock")
        return
    with serial.Serial(port, baud, timeout=1) as ser:
        ser.write(b"\r\n\r\n")
        time.sleep(2)
        ser.reset_input_buffer()
        ser.write(b"$X\n")
        time.sleep(0.2)


def on_mouse(event: int, x: int, y: int, flags, param) -> None:
    """Mouse callback storing last clicked pixel."""
    global last_click
    if event == cv2.EVENT_LBUTTONDOWN:
        last_click = (x, y)
        logger.info("click pixel=(%d, %d)", x, y)


def load_affine() -> None:
    """Load previously saved affine matrix from disk."""
    global M_affine
    if CALIB_PATH.exists():
        data = np.load(CALIB_PATH)
        M_affine = data["M"]
        logger.info("calibration loaded from %s", CALIB_PATH)


def save_affine() -> None:
    """Persist current affine matrix to disk."""
    if M_affine is not None:
        np.savez(CALIB_PATH, M=M_affine)
        logger.info("calibration saved to %s", CALIB_PATH)


def compute_affine() -> None:
    """Compute affine transform from three pixel/XY pairs."""
    global M_affine
    if len(cal_px) < 3 or len(cal_xy) < 3:
        logger.warning("need 3 calibration points")
        return
    src = np.array(cal_px[:3], dtype=np.float32)
    dst = np.array(cal_xy[:3], dtype=np.float32)
    M_affine, _ = cv2.estimateAffine2D(src, dst)
    if M_affine is None:
        logger.error("calibration failed")
    else:
        logger.info("calibration matrix\n%s", M_affine)


def px_to_xy(px: Tuple[int, int]) -> Tuple[float, float]:
    """Convert pixel coordinates to machine XY using affine matrix."""
    if M_affine is None:
        raise RuntimeError("No calibration. Press [1][2][3] then [K].")
    u, v = float(px[0]), float(px[1])
    uv1 = np.array([u, v, 1.0], dtype=np.float64)
    xy = (M_affine @ uv1.reshape(3, 1)).ravel()
    return float(xy[0]), float(xy[1])


def write_move_gcode(x: float, y: float, path: str = "move_to_target.gcode") -> str:
    """Write a small G-code file that performs a safe rapid move."""
    g = [
        "G21",
        "G90",
        f"G0 Z{SAFE_Z:.3f}",
        f"G0 X{float(x):.3f} Y{float(y):.3f} F{RAPID:.1f}",
        "M2",
    ]
    Path(path).write_text("\n".join(g) + "\n", encoding="utf-8")
    logger.info("gcode written to %s", path)
    return path


def detect_aruco_center(frame) -> Optional[Tuple[int, int]]:
    """Detect center of first ArUco marker in frame, if support is available."""
    if aruco is None:
        return None
    try:
        corners, ids, _ = aruco.detectMarkers(frame, ARUCO_DICT)
        if ids is None or len(corners) == 0:
            return None
        pts = corners[0][0]
        cx = int(np.mean(pts[:, 0]))
        cy = int(np.mean(pts[:, 1]))
        return cx, cy
    except Exception:  # pragma: no cover - detection is best effort
        return None


# --- main loop -------------------------------------------------------------
def main() -> None:  # pragma: no cover - interactive
    """Run interactive calibration and movement tool."""
    global last_click, M_affine
    load_affine()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        logger.error("No camera available")
        return

    cv2.namedWindow("Vision->CNC")
    cv2.setMouseCallback("Vision->CNC", on_mouse)

    info = (
        "[1][2][3]=add calib point -> enter X Y (mm) | [K]=solve | [S]=save | [L]=load | "
        "[T]=target click | [D]=target ArUco | [M]=MOVE | [Q]=quit"
    )
    target_px: Optional[Tuple[int, int]] = None

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        disp = frame.copy()

        for i, p in enumerate(cal_px):
            cv2.circle(disp, (int(p[0]), int(p[1])), 5, (0, 255, 0), -1)
            cv2.putText(
                disp,
                f"C{i+1}",
                (int(p[0]) + 6, int(p[1]) - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        if target_px is not None:
            cv2.drawMarker(
                disp,
                (int(target_px[0]), int(target_px[1])),
                (0, 0, 255),
                cv2.MARKER_CROSS,
                20,
                2,
            )

        if last_click is not None:
            cv2.circle(disp, (int(last_click[0]), int(last_click[1])), 5, (255, 0, 0), -1)

        cv2.putText(
            disp,
            info,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (50, 200, 255),
            2,
        )
        cv2.imshow("Vision->CNC", disp)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), ord("Q")):
            break

        elif key in (ord("1"), ord("2"), ord("3")):
            if last_click is None:
                info = "Klikni do obrazu najprv."
                continue
            try:
                raw = input("Zadaj X Y (mm): ").strip()
                xs, ys = raw.split()
                x, y = float(xs), float(ys)
                cal_px.append(tuple(last_click))
                cal_xy.append((x, y))
                info = f"Kalibračný bod uložený ({len(cal_px)})"
            except Exception as exc:  # pragma: no cover - user input
                info = f"Zlé X Y: {exc}"

        elif key in (ord("k"), ord("K")):
            compute_affine()
            info = "Kalibrácia hotová" if M_affine is not None else "Kalibrácia zlyhala"

        elif key in (ord("s"), ord("S")):
            save_affine()
            info = "Kalibrácia uložená"

        elif key in (ord("l"), ord("L")):
            load_affine()
            info = "Kalibrácia načítaná"

        elif key in (ord("t"), ord("T")):
            if last_click is None:
                info = "Klikni na cieľ."
            else:
                target_px = last_click
                info = f"Cieľ=klik {target_px}"

        elif key in (ord("d"), ord("D")):
            center = detect_aruco_center(frame)
            if center is None:
                info = "Aruco nie je k dispozícii alebo marker mimo záber."
            else:
                target_px = center
                info = f"Cieľ=Aruco {center}"

        elif key in (ord("m"), ord("M")):
            try:
                if target_px is None:
                    info = "Najprv nastav cieľ [T]/[D]."
                    continue
                if M_affine is None:
                    info = "Najprv kalibrácia [1][2][3] a [K]."
                    continue
                x, y = px_to_xy(target_px)
                logger.info("MOVE pixel %s -> XY (%.3f, %.3f)", target_px, x, y)
                gpath = write_move_gcode(x, y)
                grbl_unlock(PORT, BAUD)
                stream_gcode_to_grbl(gpath, PORT, baud=BAUD)
                info = f"Poslané: X{x:.2f} Y{y:.2f}"
            except Exception as exc:  # pragma: no cover - hardware interaction
                info = f"ERROR: {exc}"
                logger.exception("move failed")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":  # pragma: no cover - script entry
    main()

