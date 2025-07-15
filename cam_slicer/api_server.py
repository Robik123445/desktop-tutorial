from pathlib import Path
 from typing import List, Tuple, Optional
 
 from fastapi import FastAPI, HTTPException, Header, Depends, Request
 from fastapi.responses import PlainTextResponse
 from fastapi.middleware.cors import CORSMiddleware
 from pydantic import BaseModel
 
 from cam_slicer.logging_config import setup_logging
 from cam_slicer import (
     load_plugins,
     get_all_plugins,
     get_plugin,
     toolpath_to_gcode,
     execute_plugin,
     stream_robotic_toolpath,
     ArmKinematicProfile,
     optimize_robotic_trajectory,
 )
 from cam_slicer.sender import send_gcode_over_serial, list_available_ports
 from cam_slicer.ai.analyzers import (
     feedrate_advisor,
     trajectory_cleaner,
     surface_comparator,
 )
 from cam_slicer.gcode_stats import parse_gcode_text, compute_gcode_stats
+from cam_slicer.toolpath_simulator import simulate_toolpath
 
 setup_logging()
 API_TOKEN = os.environ.get("API_TOKEN", "changeme")
 
 app = FastAPI(title="CAM Slicer API")
 app.add_middleware(
     CORSMiddleware,
     allow_origins=["*"],
     allow_methods=["*"],
     allow_headers=["*"],
 )
 
 
 @app.middleware("http")
 async def log_requests(request: Request, call_next):
     logging.info("%s %s", request.method, request.url.path)
     return await call_next(request)
 
 
 def verify_token(x_access_token: str = Header(...)) -> str:
     if x_access_token != API_TOKEN:
         logging.warning("Unauthorized API access")
         raise HTTPException(status_code=403, detail="Invalid token")
     return x_access_token
 
diff --git a/cam_slicer/api_server.py b/cam_slicer/api_server.py
index e83586c47ffd51950f3b1df5c3642e596abae283..878ae3281406f4e013d01328cf357a89d1e26d79 100644
--- a/cam_slicer/api_server.py
+++ b/cam_slicer/api_server.py
@@ -74,50 +75,55 @@ class StreamRequest(BaseModel):
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
 
 
 class GcodeStatsRequest(BaseModel):
     gcode: str
 
 
+class SimulateRequest(BaseModel):
+    gcode: str
+    include_plot: bool = False
+
+
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
diff --git a/cam_slicer/api_server.py b/cam_slicer/api_server.py
index e83586c47ffd51950f3b1df5c3642e596abae283..878ae3281406f4e013d01328cf357a89d1e26d79 100644
--- a/cam_slicer/api_server.py
+++ b/cam_slicer/api_server.py
@@ -225,34 +231,40 @@ def list_ports(token: str = Depends(verify_token)) -> list[str]:
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
 
 
 @app.post("/gcode/stats")
 def gcode_stats(req: GcodeStatsRequest, token: str = Depends(verify_token)) -> dict:
     """Return statistics for provided G-code text."""
     lines = parse_gcode_text(req.gcode)
     return compute_gcode_stats(lines)
 
 
+@app.post("/simulate")
+def simulate(req: SimulateRequest, token: str = Depends(verify_token)) -> dict:
+    """Parse G-code and optionally return a PNG preview."""
+    return simulate_toolpath(req.gcode, include_plot=req.include_plot)
+
+
 def create_app() -> FastAPI:
     """Return configured FastAPI application."""
     return app
 
 
 if __name__ == "__main__":  # pragma: no cover
     import uvicorn
 
     uvicorn.run(app, host="0.0.0.0", port=8000)
