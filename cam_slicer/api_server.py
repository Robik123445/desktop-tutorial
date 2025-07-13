from typing import List, Tuple, Optional
from fastapi import FastAPI, HTTPException, Header, Depends, Request
from pydantic import BaseModel
import os
import logging

from cam_slicer.logging_config import setup_logging
from cam_slicer import (
    load_plugins,
    get_all_plugins,
    get_plugin,
    toolpath_to_gcode,
    optimize_toolpath,
    stream_robotic_toolpath,
    ArmKinematicProfile,
    optimize_robotic_trajectory,
    execute_plugin,
)

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


class StreamRequest(BaseModel):
    points: List[Tuple[float, float, float]]
    port: str
    baud: int = 115200
    profile: Optional[dict] = None


class TrajectoryRequest(BaseModel):
    points: List[Tuple[float, float, float]]
    profile: Optional[dict] = None


@app.on_event("startup")
def startup_event() -> None:
    load_plugins()
    logging.info("API server started and plugins loaded")


@app.get("/plugins")
def list_plugins(token: str = Depends(verify_token)) -> List[dict]:
    """Return available plugins."""
    return get_all_plugins()


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


@app.post("/export")
def export_gcode(tp: Toolpath, token: str = Depends(verify_token)) -> List[str]:
    """Convert a toolpath to G-code."""
    gcode = toolpath_to_gcode(tp.points)
    return gcode


@app.post("/optimize")
def optimize(tp: Toolpath, token: str = Depends(verify_token)) -> List[Tuple[float, float, float]]:
    """Optimize a toolpath."""
    return optimize_toolpath(tp.points)


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


def create_app() -> FastAPI:
    """Return configured FastAPI application."""
    return app


if __name__ == "__main__":  # pragma: no cover
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
