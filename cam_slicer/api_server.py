from typing import List, Tuple, Optional
 from fastapi import FastAPI, HTTPException, Header, Depends, Request
+from fastapi.responses import PlainTextResponse
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
diff --git a/cam_slicer/api_server.py b/cam_slicer/api_server.py
index 18fb4ba0af18b08eb0aec95762cde839dae7ee77..f256fa4218523e5911cc24a603d867096a2f6fa4 100644
--- a/cam_slicer/api_server.py
+++ b/cam_slicer/api_server.py
@@ -64,55 +65,56 @@ def startup_event() -> None:
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
 
 
-@app.post("/export")
-def export_gcode(tp: Toolpath, token: str = Depends(verify_token)) -> List[str]:
-    """Convert a toolpath to G-code."""
-    gcode = toolpath_to_gcode(tp.points)
-    return gcode
+@app.post("/export", response_class=PlainTextResponse)
+def export_gcode(tp: Toolpath, token: str = Depends(verify_token)) -> str:
+    """Convert a toolpath to G-code and return plain text."""
+    logging.info("Export %d points to G-code", len(tp.points))
+    lines = toolpath_to_gcode(tp.points)
+    return "\n".join(lines)
 
 
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
