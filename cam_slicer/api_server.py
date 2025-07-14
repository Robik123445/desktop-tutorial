diff --git a/cam_slicer/api_server.py b/cam_slicer/api_server.py
index 8bc11a51b3996eaaca35d73d571ce7229471bb8e..036410a8c03a43f81f782ba40c800dadf889462c 100644
--- a/cam_slicer/api_server.py
+++ b/cam_slicer/api_server.py
@@ -1,48 +1,48 @@
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
-from cam_slicer.sender import send_gcode_over_serial
+from cam_slicer.sender import send_gcode_over_serial, list_available_ports
 
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
diff --git a/cam_slicer/api_server.py b/cam_slicer/api_server.py
index 8bc11a51b3996eaaca35d73d571ce7229471bb8e..036410a8c03a43f81f782ba40c800dadf889462c 100644
--- a/cam_slicer/api_server.py
+++ b/cam_slicer/api_server.py
@@ -153,47 +153,58 @@ def stream_robotic(req: StreamRequest, token: str = Depends(verify_token)) -> di
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
 
 
+@app.get("/ports")
+def list_ports(token: str = Depends(verify_token)) -> list[str]:
+    """Return available serial port names."""
+    logging.info("Listing serial ports")
+    try:
+        return list_available_ports()
+    except Exception as exc:  # pragma: no cover - runtime errors
+        logging.error("Failed to list ports: %s", exc)
+        raise HTTPException(status_code=500, detail=str(exc))
+
+
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
