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
    """Request body for /plugins/run."""
    name: str
    toolpath: Optional[List[Tuple[float, float, float]]] = None
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

@app.on_event("startup")
def startup_event() -> None:
    load_plugins()
    logging.info("API server started and plugins loaded")

@app.get("/plugins")
def list_plugins(token: str = Depends(verify_token)) -> List[dict]:
    """Return available plugins with name and description."""
    plugins = get_all_plugins()
    return [{"name": p["name"], "description": p["description"]} for p in plugins]

@app.post("/plugins/{name}")
def run_plugin(name: str, req: PluginRunRequest, token: str = Depends(verify_token)) -> List[Tuple[float, float, float]]:
    """Run a plugin and return its result."""
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
        logging.error("Plugin %s failed: %s", name, exc)
        raise HTTPException(status_code=500, detail=str(exc))
    return result

@app.post("/plugins/run")
def run_plugin_generic(req: PluginExecRequest, token: str = Depends(verify_token)):
    """Run a plugin specified in the request body and return its result."""
    plugin = get_plugin(req.name)
    if not plugin:
        raise HTTPException(status_code=404, detail="Plugin not found")
    args = req.args or []
    try:
        if req.toolpath is not None:
            result = execute_plugin(req.name, [req.toolpath])
        else:
            result = execute_plugin(req.name, args)
    except Exception as exc:
        logging.error("Plugin %s failed: %s", req.name, exc)
        raise HTTPException(status_code=500, detail=str(exc))
    return result

@app.post("/export", response_class=PlainTextResponse)
def export_gcode(tp: Toolpath, token: str = Depends(verify_token)) -> str:
    """Convert a toolpath to G-code and return plain text."""
    logging.info("Export %d points to G-code", len(tp.points))
    lines = toolpath_to_gcode(tp.points)
    return "\n".join(lines)

ANALYZERS = {
    "Feedrate Advisor": feedrate_advisor,
    "Trajectory Cleaner": trajectory_cleaner,
    "Surface Comparator": surface_comparator,
    "ML Speed Optimizer": ml_speed_optimizer,
    "Plugin Optimizer": plugin_optimizer,
    "feedrate_advisor": feedrate_advisor,
    "trajectory_cleaner": trajectory_cleaner,
    "surface_comparator": surface_comparator,
    "ml_speed_optimizer": ml_speed_optimizer,
    "plugin_optimizer": plugin_optimizer,
}

@app.post("/optimize")
def optimize(req: OptimizeRequest, token: str = Depends(verify_token)) -> dict:
    """Run selected analysis on a toolpath."""
    if req.analyzer.startswith("plugin:"):
        name = req.analyzer.split(":", 1)[1]
        logging.info("Running plugin optimizer %s", name)
        return plugin_optimizer(req.points, name)
    if req.analyzer.startswith("ml:"):
        logging.info("Running ML optimizer %s", req.analyzer)
        return ml_speed_optimizer(req.points)
    func = ANALYZERS.get(req.analyzer)
    if not func:
        raise HTTPException(status_code=400, detail="Unknown analyzer")
    logging.info("Running %s on %d points", req.analyzer, len(req.points))
    try:
        if func is feedrate_advisor:
            return func(req.points, tool=req.tool, material=req.material)
        return func(req.points)
    except Exception as exc:  # pragma: no cover - runtime errors
        logging.error("Analyzer %s failed: %s", req.analyzer, exc)
        raise HTTPException(status_code=500, detail=str(exc))

@app.post("/robot_optimize")
def robot_optimize(req: TrajectoryRequest, token: str = Depends(verify_token)) -> dict:
    """Optimize robotic trajectory and return suggestions."""
    profile = (
        ArmKinematicProfile(**req.profile)
        if req.profile
        else ArmKinematicProfile(name="basic")
    )
    optimized, warnings = optimize_robotic_trajectory(req.points, profile)
    return {"points": optimized, "warnings": warnings}

@app.post("/stream_robotic")
def stream_robotic(req: StreamRequest, token: str = Depends(verify_token)) -> dict:
    """Stream robotic toolpath to the machine."""
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
    """Send G-code text over a serial port and return log output."""
    logging.info("Sending G-code to %s", req.port)
    try:
        log_output = send_gcode_over_serial(req.gcode, req.port)
        status = "ok"
    except Exception as exc:  # pragma: no cover - runtime errors
        logging.error("Serial send failed: %s", exc)
        log_output = str(exc)
        status = "error"
    return {"status": status, "log": log_output}

@app.post("/probe_heightmap")
def probe_heightmap_api(req: ProbeRequest, token: str = Depends(verify_token)) -> dict:
    """Probe a heightmap and return measured points."""
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
    except Exception as exc:  # pragma: no cover - runtime errors
        logging.error("Heightmap probe failed: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))

@app.get("/ports")
def list_ports(token: str = Depends(verify_token)) -> list[str]:
    """Return available serial port names."""
    logging.info("Listing serial ports")
    try:
        return list_available_ports()
    except Exception as exc:  # pragma: no cover - runtime errors
        logging.error("Failed to list ports: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))

@app.get("/logs/central.log", response_class=PlainTextResponse)
def get_central_log(token: str = Depends(verify_token)) -> str:
    """Return contents of the main log file."""
    log_path = Path("logs/central.log")
    if not log_path.exists():
        raise HTTPException(status_code=404, detail="Log not found")
    try:
        return log_path.read_text()
    except Exception as exc:  # pragma: no cover - file errors
        logging.error("Failed to read central log: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))

def create_app() -> FastAPI:
    """Return configured FastAPI application."""
    return app

if __name__ == "__main__":  # pragma: no cover
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
