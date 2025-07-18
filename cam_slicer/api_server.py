from typing import List, Tuple, Optional
from fastapi import FastAPI, HTTPException, Header, Depends, Request
from fastapi.responses import PlainTextResponse
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

setup_logging()
API_TOKEN = os.environ.get("API_TOKEN", "changeme")

app = FastAPI(title="CAM Slicer API")

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
    profile: Optional[dict] = None

class TrajectoryRequest(BaseModel):
    points: List[Tuple[float, float, float]]
    profile: Optional[dict] = None

class OptimizeRequest(BaseModel):
    analyzer: str
    points: List[Tuple[float, float, float]]
    tool: Optional[str] = None
    material: Optional[str] = None

class SendRequest(BaseModel):
    gcode: str
    port: str

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

@app.on_event("startup")
def startup_event() -> None:
    load_plugins()
    logging.info("API server started and plugins loaded")

@app.get("/plugins")
def list_plugins(token: str = Depends(verify_token)) -> List[dict]:
    plugins = get_all_plugins()
    return [{"name": p["name"], "description": p["description"]} for p in plugins]

@app.post("/plugins/{name}")
def run_plugin(name: str, req: PluginRunRequest, token: str = Depends(verify_token)) -> List[Tuple[float, float, float]]:
    plugin = get_plugin(name)
    if not plugin:
        raise HTTPException(status_code=404, detail="Plugin not found")
    args = req.args or []
    toolpath = req.toolpath
    try:
        if toolpath is not None:
            result = execute_plugin(name, [toolpath])
        else:
            result = execute_plugin(name, args)
    except Exception as exc:
        logging.error("Plugin run failed: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))
    return result

@app.post("/stream_robotic")
def stream_robotic(req: StreamRequest, token: str = Depends(verify_token)) -> dict:
    profile = (
        ArmKinematicProfile(**req.profile)
        if req.profile
        else ArmKinematicProfile(name="basic")
    )
    try:
        stream_robotic_toolpath(req.points, profile, req.port, req.baud)
    except Exception as exc:
        logging.error("Streaming failed: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))
    return {"status": "ok"}

@app.post("/send")
def send_gcode(req: SendRequest, token: str = Depends(verify_token)) -> dict:
    logging.info("Sending G-code to %s", req.port)
    try:
        log_output = send_gcode_over_serial(req.gcode, req.port)
        status = "ok"
    except Exception as exc:
        logging.error("Serial send failed: %s", exc)
        log_output = str(exc)
        status = "error"
    return {"status": status, "log": log_output}

@app.post("/heightmap")
def apply_heightmap_api(
    req: HeightmapRequest, token: str = Depends(verify_token)
) -> dict:
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
