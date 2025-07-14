diff --git a/cam_slicer/sender/__init__.py b/cam_slicer/sender/__init__.py
index c6b862e33167934f9612f66b3c0012fe68850312..37fc463b1580dab10e348ff9f6a445fbb2bb8746 100644
--- a/cam_slicer/sender/__init__.py
+++ b/cam_slicer/sender/__init__.py
@@ -1,25 +1,26 @@
 """Serial streaming utilities."""
 
-from .serial_stream import stream_gcode_live
+from .serial_stream import stream_gcode_live, send_gcode_over_serial
 from .serial_streamer import stream_gcode_to_grbl
 from .grbl_streamer import stream_gcode_interactive
 from .live_gcode_streamer import LiveGcodeStreamer
 from .feedback_streamer import stream_gcode_with_feedback
 from .job_recovery import (
     stream_with_recovery,
     resume_job,
     save_checkpoint,
     load_checkpoint,
 )
 
 __all__ = [
     "stream_gcode_live",
     "stream_gcode_to_grbl",
     "stream_gcode_interactive",
     "LiveGcodeStreamer",
     "stream_with_recovery",
     "resume_job",
     "save_checkpoint",
     "load_checkpoint",
     "stream_gcode_with_feedback",
+    "send_gcode_over_serial",
 ]
