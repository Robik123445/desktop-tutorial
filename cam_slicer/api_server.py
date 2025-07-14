diff --git a/cam_slicer/api_server.py b/cam_slicer/api_server.py
index 036410a8c03a43f81f782ba40c800dadf889462c..d0ff4ece060cd1549f127fae03022f249a497e26 100644
--- a/cam_slicer/api_server.py
+++ b/cam_slicer/api_server.py
@@ -26,103 +26,129 @@ setup_logging()
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
 
 
+class PluginExecRequest(BaseModel):
+    """Request body for /plugins/run."""
+    name: str
+    toolpath: Optional[List[Tuple[float, float, float]]] = None
+    args: Optional[List[str]] = None
+
+
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
 
 
 @app.on_event("startup")
 def startup_event() -> None:
     load_plugins()
     logging.info("API server started and plugins loaded")
 
 
 @app.get("/plugins")
 def list_plugins(token: str = Depends(verify_token)) -> List[dict]:
-    """Return available plugins."""
-    return get_all_plugins()
+    """Return available plugins with name and description."""
+    plugins = get_all_plugins()
+    return [{"name": p["name"], "description": p["description"]} for p in plugins]
 
 
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
 
 
+@app.post("/plugins/run")
+def run_plugin_generic(req: PluginExecRequest, token: str = Depends(verify_token)):
+    """Run a plugin specified in the request body and return its result."""
+    plugin = get_plugin(req.name)
+    if not plugin:
+        raise HTTPException(status_code=404, detail="Plugin not found")
+    args = req.args or []
+    try:
+        if req.toolpath is not None:
+            result = execute_plugin(req.name, [req.toolpath])
+        else:
+            result = execute_plugin(req.name, args)
+    except Exception as exc:
+        logging.error("Plugin %s failed: %s", req.name, exc)
+        raise HTTPException(status_code=500, detail=str(exc))
+    return result
+
+
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
     "feedrate_advisor": feedrate_advisor,
     "trajectory_cleaner": trajectory_cleaner,
     "surface_comparator": surface_comparator,
 }
 
 
 @app.post("/optimize")
 def optimize(req: OptimizeRequest, token: str = Depends(verify_token)) -> dict:
     """Run selected analysis on a toolpath."""
     func = ANALYZERS.get(req.analyzer)
     if not func:
         raise HTTPException(status_code=400, detail="Unknown analyzer")
     logging.info("Running %s on %d points", req.analyzer, len(req.points))
