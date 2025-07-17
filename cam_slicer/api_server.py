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
)
from cam_slicer.sender import send_gcode_over_serial, list_available_ports
from cam_slicer.probing import probe_heightmap

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
    args: Optional[List[str]] = None

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
        return result
    except Exception as exc:
        logging.error("Plugin execution failed: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))

@app.post("/plugins/run")
def run_plugin_generic(req: PluginRunRequest, token: str = Depends(verify_token)) -> List[Tuple[float, float, float]]:
    name = req.args[0] if req.args else None
    if not name:
        raise HTTPException(status_code=422, detail="Missing plugin name")
    plugin = get_plugin(name)
    if not plugin:
        raise HTTPException(status_code=404, detail="Plugin not found")
    toolpath = req.toolpath
    try:
        if toolpath is not None:
            result = execute_plugin(name, [toolpath])
        else:
            result = execute_plugin(name, req.args[1:])
        return result
    except Exception as exc:
        logging.error("Plugin execution failed: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))

@app.post("/robot_optimize")
def robot_optimize(req: TrajectoryRequest, token: str = Depends(verify_token)) -> dict:
    profile = ArmKinematicProfile(**req.profile) if req.profile else ArmKinematicProfile(name="basic")
    try:
        result = optimize_robotic_trajectory(req.points, profile)
        return result
    except Exception as exc:
        logging.error("Robotic optimize failed: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))

@app.post("/export")
def export_gcode_api(req: Toolpath, token: str = Depends(verify_token)) -> PlainTextResponse:
    try:
        gcode = toolpath_to_gcode(req.points, None)
        return PlainTextResponse(gcode)
    except Exception as exc:
        logging.error("G-code export failed: %s", exc)
        raise HTTPException(status_code=422, detail=str(exc))

@app.post("/optimize")
def optimize_toolpath_api(req: OptimizeRequest, token: str = Depends(verify_token)) -> dict:
    if req.analyzer == "Trajectory Cleaner":
        result = trajectory_cleaner(req.points)
        return result
    elif req.analyzer == "Feedrate Advisor":
        result = feedrate_advisor(req.points)
        return result
    elif req.analyzer == "Surface Comparator":
        result = surface_comparator(req.points)
        return result
    else:
        raise HTTPException(status_code=400, detail="Unknown analyzer")

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

@app.get("/logs/central.log", response_class=PlainTextResponse)
def get_central_log(token: str = Depends(verify_token)) -> str:
    log_path = Path("logs/central.log")
    if not log_path.exists():
        raise HTTPException(status_code=404, detail="Log not found")
    try:
        return log_path.read_text()
    except Exception as exc:
        logging.error("Failed to read central log: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))

def create_app() -> FastAPI:
    return app
