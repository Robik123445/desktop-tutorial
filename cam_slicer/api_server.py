from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Optional
import json
import numpy as np

from fastapi import Depends, FastAPI, Header, HTTPException, Request
from fastapi.responses import PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from cam_slicer.logging_config import setup_logging
from cam_slicer.plugin_manager import (
    execute_plugin,
    get_all_plugins,
    get_plugin,
    load_plugins,
)
from cam_slicer.core.gcode_export import toolpath_to_gcode
from cam_slicer.robotics.exporter import stream_robotic_toolpath
from cam_slicer.robotics.interface import ArmKinematicProfile
from cam_slicer.robotics.trajectory_optimizer import optimize_robotic_trajectory
from cam_slicer.ai.analyzers import (
    feedrate_advisor,
    ml_speed_optimizer,
    plugin_optimizer,
    surface_comparator,
    trajectory_cleaner,
)
from cam_slicer.sender import list_available_ports
from cam_slicer.probing import probe_heightmap
from cam_slicer.heightmap import HeightMap, apply_heightmap_to_gcode
from cam_slicer.vision.camera_router import router as vision_router

app = FastAPI(title="CAM Slicer API")

API_TOKEN = "changeme"

setup_logging()
logger = logging.getLogger(__name__)
_log_path = Path("logs/log.txt")
if not any(
    isinstance(h, logging.FileHandler)
    and getattr(h, "baseFilename", "") == str(_log_path)
    for h in logging.getLogger().handlers
):
    _log_path.parent.mkdir(exist_ok=True)
    fh = logging.FileHandler(_log_path)
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logging.getLogger().addHandler(fh)

def create_app() -> FastAPI:
    """Return configured FastAPI instance."""
    global API_TOKEN
    setup_logging()
    API_TOKEN = os.environ.get("API_TOKEN", API_TOKEN)

    if not any(getattr(r, "path", "").startswith("/logs") for r in app.routes):
        logs_dir = Path("logs")
        logs_dir.mkdir(exist_ok=True)
        app.mount("/logs", StaticFiles(directory=str(logs_dir)), name="logs")
    if not any(getattr(r, "path", "") == "/vision/cameras" for r in app.routes):
        app.include_router(vision_router)

    return app

@app.middleware("http")
async def log_requests(request: Request, call_next):
    logging.info("%s %s", request.method, request.url.path)
    return await call_next(request)

def verify_token(x_access_token: str = Header(...)) -> str:
    if x_access_token != API_TOKEN:
        logging.warning("Unauthorized API access")
        raise HTTPException(status_code=403, detail="Invalid token")
    return x_access_token

class Toolpath(BaseModel):
    """Toolpath data container."""

    points: list[tuple[float, float, float]]


class PluginRunRequest(BaseModel):
    """Request schema for running a plugin."""

    toolpath: list[tuple[float, float, float]] | None = None
    args: list[str] | None = None

class PluginExecRequest(BaseModel):
    pass

class StreamRequest(BaseModel):
    """Request for streaming a toolpath over serial."""

    points: list[tuple[float, float, float]]
    port: str
    baud: int = 115200

class ProbeRequest(BaseModel):
    """Request configuration for probing a heightmap."""

    x_start: float
    x_end: float
    y_start: float
    y_end: float
    step: float
    port: str | None = None
    baud: int = 115200

class HeightmapRequest(BaseModel):
    """Request payload for applying a heightmap to G-code."""
    gcode: str
    heightmap: str
    format: str = "csv"

@app.post("/heightmap")
def apply_heightmap_api(
    req: HeightmapRequest, token: str = Depends(verify_token)
) -> dict:
    """Apply a heightmap to raw G-code and return adjusted text."""
    logging.info("Applying heightmap to G-code")
    try:
        hm = HeightMap.from_text(req.heightmap, fmt=req.format)
        gcode = apply_heightmap_to_gcode(req.gcode, hm)
    except ValueError as exc:
        logging.error("Invalid heightmap: %s", exc)
        raise HTTPException(status_code=400, detail=str(exc))
    except Exception as exc:
        logging.error("Failed to apply heightmap: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))
    return {"gcode": gcode}

@app.post("/probe_heightmap")
def probe_heightmap_api(req: ProbeRequest, token: str = Depends(verify_token)) -> dict:
    try:
        zmap = probe_heightmap(
            (req.x_start, req.x_end),
            (req.y_start, req.y_end),
            req.step,
            port=req.port,
            baud=req.baud,
            save_path="logs/heightmap.json",
        )
        return {"points": zmap.points}
    except Exception as exc:
        logging.error("Heightmap probe failed: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))

@app.get("/ports")
def list_ports(token: str = Depends(verify_token)) -> list[str]:
    logging.info("Listing serial ports")
    try:
        return list_available_ports()
    except Exception as exc:
        logging.error("Failed to list ports: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))

# --- calibration helpers ---
_CALIB_PATH = Path("vision_affine.npz")


def _save_affine(M: np.ndarray) -> None:
    """Persist affine matrix to disk."""
    _CALIB_PATH.parent.mkdir(exist_ok=True)
    np.savez(_CALIB_PATH, M=M)


def _load_affine() -> Optional[np.ndarray]:
    """Load affine matrix from disk if it exists."""
    if _CALIB_PATH.exists():
        data = np.load(_CALIB_PATH)
        return data["M"]
    return None


def _px_to_xy(px_u: float, px_v: float, M: np.ndarray) -> tuple[float, float]:
    """Convert pixel coordinates to machine XY using matrix M."""
    uv1 = np.array([px_u, px_v, 1.0], dtype=np.float64)
    xy = (M @ uv1.reshape(3, 1)).ravel()
    return float(xy[0]), float(xy[1])


def _write_move_gcode(x: float, y: float, path: Path) -> Path:
    """Create a small G-code file for safe rapid move."""
    lines = [
        "G21",
        "G90",
        "G0 Z10.000",
        f"G0 X{float(x):.3f} Y{float(y):.3f} F1500.0",
        "M2",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


# --- Pydantic models ---
class CalibPoint(BaseModel):
    pixel_u: float
    pixel_v: float
    machine_x: float
    machine_y: float


class VisionCalibIn(BaseModel):
    points: list[CalibPoint] = Field(min_items=3, max_items=3, description="Exactly 3 points.")


class VisionCalibOut(BaseModel):
    matrix: list[list[float]]


class VisionMoveIn(BaseModel):
    pixel_u: float
    pixel_v: float
    port: str = "/dev/ttyUSB0"
    baud: int = 115200


class VisionMoveOut(BaseModel):
    x: float
    y: float
    gcode_path: str


# --- routes ---
@app.post("/vision/calibrate", response_model=VisionCalibOut, dependencies=[Depends(verify_token)])
def vision_calibrate(payload: VisionCalibIn):
    """Compute and store affine calibration from three pixel/XY pairs."""
    try:
        import cv2
        src = np.array([[p.pixel_u, p.pixel_v] for p in payload.points], dtype=np.float32)
        dst = np.array([[p.machine_x, p.machine_y] for p in payload.points], dtype=np.float32)
        M, _ = cv2.estimateAffine2D(src, dst)
        if M is None:
            raise HTTPException(status_code=400, detail="Calibration failed")
        logger.info("vision calibration %s", json.dumps(M.tolist()))
        _save_affine(M)
        return {"matrix": M.tolist()}
    except HTTPException:
        raise
    except Exception as e:  # pragma: no cover - calibration failures
        logger.exception("vision_calibrate error")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/vision/calibration", response_model=VisionCalibOut, dependencies=[Depends(verify_token)])
def vision_get_calibration():
    """Return previously stored affine matrix."""
    M = _load_affine()
    if M is None:
        raise HTTPException(status_code=404, detail="No calibration")
    return {"matrix": M.tolist()}


@app.post("/vision/move_to", response_model=VisionMoveOut, dependencies=[Depends(verify_token)])
def vision_move_to(payload: VisionMoveIn):
    """Convert pixel position to machine move and stream to GRBL."""
    try:
        M = _load_affine()
        if M is None:
            raise HTTPException(status_code=400, detail="No calibration. Call /vision/calibrate first.")
        x, y = _px_to_xy(payload.pixel_u, payload.pixel_v, M)
        gpath = Path("move_to_target_api.gcode")
        _write_move_gcode(x, y, gpath)
        try:
            import serial, time
            with serial.Serial(payload.port, payload.baud, timeout=1) as ser:
                ser.write(b"\r\n\r\n"); time.sleep(2); ser.reset_input_buffer()
                ser.write(b"$X\n"); time.sleep(0.2)
        except Exception:  # pragma: no cover - serial optional
            pass
        from cam_slicer.sender.serial_streamer import stream_gcode_to_grbl
        stream_gcode_to_grbl(str(gpath), payload.port, baud=payload.baud)
        return {"x": x, "y": y, "gcode_path": str(gpath)}
    except HTTPException:
        raise
    except Exception as e:  # pragma: no cover - hardware failures
        logger.exception("vision_move_to error")
        raise HTTPException(status_code=500, detail=str(e))


__all__ = ["app", "create_app"]
