diff --git a/cam_slicer/api_server.py b/cam_slicer/api_server.py
index 12f9aae39ef3868a66cd5a36c2d2d0d2f18f89fe..8bc11a51b3996eaaca35d73d571ce7229471bb8e 100644
--- a/cam_slicer/api_server.py
+++ b/cam_slicer/api_server.py
@@ -1,28 +1,29 @@
 from typing import List, Tuple, Optional
 from fastapi import FastAPI, HTTPException, Header, Depends, Request
 from fastapi.responses import PlainTextResponse
+from pathlib import Path
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
 from cam_slicer.sender import send_gcode_over_serial
 
 setup_logging()
 API_TOKEN = os.environ.get("API_TOKEN", "changeme")
 
 app = FastAPI(title="CAM Slicer API")
 
diff --git a/cam_slicer/api_server.py b/cam_slicer/api_server.py
index 12f9aae39ef3868a66cd5a36c2d2d0d2f18f89fe..8bc11a51b3996eaaca35d73d571ce7229471bb8e 100644
--- a/cam_slicer/api_server.py
+++ b/cam_slicer/api_server.py
@@ -152,34 +153,47 @@ def stream_robotic(req: StreamRequest, token: str = Depends(verify_token)) -> di
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
 
 
+@app.get("/logs/central.log", response_class=PlainTextResponse)
+def get_central_log(token: str = Depends(verify_token)) -> str:
+    """Return contents of the main log file."""
+    log_path = Path("logs/central.log")
+    if not log_path.exists():
+        raise HTTPException(status_code=404, detail="Log not found")
+    try:
+        return log_path.read_text()
+    except Exception as exc:  # pragma: no cover - file errors
+        logging.error("Failed to read central log: %s", exc)
+        raise HTTPException(status_code=500, detail=str(exc))
+
+
 def create_app() -> FastAPI:
     """Return configured FastAPI application."""
     return app
 
 
 if __name__ == "__main__":  # pragma: no cover
     import uvicorn
 
     uvicorn.run(app, host="0.0.0.0", port=8000)
