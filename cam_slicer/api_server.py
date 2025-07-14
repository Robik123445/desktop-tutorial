diff --git a/cam_slicer/api_server.py b/cam_slicer/api_server.py
index f256fa4218523e5911cc24a603d867096a2f6fa4..68c1bd190da4801f4901f2bfc981d8342fd8b8e8 100644
--- a/cam_slicer/api_server.py
+++ b/cam_slicer/api_server.py
@@ -1,83 +1,89 @@
 from typing import List, Tuple, Optional
 from fastapi import FastAPI, HTTPException, Header, Depends, Request
 from fastapi.responses import PlainTextResponse
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
+from cam_slicer.sender import send_gcode_over_serial
 
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
 
 
+class SendRequest(BaseModel):
+    gcode: str
+    port: str
+
+
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
diff --git a/cam_slicer/api_server.py b/cam_slicer/api_server.py
index f256fa4218523e5911cc24a603d867096a2f6fa4..68c1bd190da4801f4901f2bfc981d8342fd8b8e8 100644
--- a/cam_slicer/api_server.py
+++ b/cam_slicer/api_server.py
@@ -107,34 +113,48 @@ def robot_optimize(req: TrajectoryRequest, token: str = Depends(verify_token)) -
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
 
 
+@app.post("/send")
+def send_gcode(req: SendRequest, token: str = Depends(verify_token)) -> dict:
+    """Send G-code text over a serial port and return log output."""
+    logging.info("Sending G-code to %s", req.port)
+    try:
+        log_output = send_gcode_over_serial(req.gcode, req.port)
+        status = "ok"
+    except Exception as exc:  # pragma: no cover - runtime errors
+        logging.error("Serial send failed: %s", exc)
+        log_output = str(exc)
+        status = "error"
+    return {"status": status, "log": log_output}
+
+
 def create_app() -> FastAPI:
     """Return configured FastAPI application."""
     return app
 
 
 if __name__ == "__main__":  # pragma: no cover
     import uvicorn
 
     uvicorn.run(app, host="0.0.0.0", port=8000)
