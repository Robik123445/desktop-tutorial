from typing import List, Tuple, Optional
from fastapi import FastAPI, HTTPException, Header, Depends, Request
from fastapi.responses import PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pathlib import Path
from pydantic import BaseModel
import os
import logging

from cam_slicer.logging_config import setup_logging
from cam_slicer import (
    load_plugins,
    get_all_plugins,
    get_plugin,
    toolpath_to_gcode,
    stream_robotic_toolpath,
    ArmKinematicProfile,
    optimize_robotic_trajectory,
    execute_plugin,
    feedrate_advisor,
    trajectory_cleaner,
    surface_comparator,
    plugin_optimizer,
    ml_speed_optimizer,
)
from cam_slicer.sender import send_gcode_over_serial, list_available_ports
from cam_slicer.probing import probe_heightmap
from cam_slicer.heightmap import HeightMap, apply_heightmap_to_gcode

app = FastAPI(title="CAM Slicer API")

API_TOKEN = "changeme"

def create_app() -> FastAPI:
    """Return configured FastAPI instance."""
    global API_TOKEN
    setup_logging()
    API_TOKEN = os.environ.get("API_TOKEN", API_TOKEN)

    if not any(getattr(r, "path", "").startswith("/logs") for r in app.routes):
        logs_dir = Path("logs")
        logs_dir.mkdir(exist_ok=True)
        app.mount("/logs", StaticFiles(directory=str(logs_dir)), name="logs")

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
    points: List[Tuple[float, float, float]]

class PluginRunRequest(BaseModel):
    toolpath: Optional[List[Tuple[float, float, float]]] = None
    args: Optional[List[str]] = None

class PluginExecRequest(BaseModel):
    pass

class StreamRequest(BaseModel):
    points: List[Tuple[float, float, float]]
    port: str
    baud: int = 115200

class ProbeRequest(BaseModel):
    x_start: float
    x_end: float
    y_start: float
    y_end: float
    step: float
    port: Optional[str] = None
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

__all__ = ["app", "create_app"]
