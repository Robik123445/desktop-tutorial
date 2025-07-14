diff --git a/cam_slicer/sender/serial_stream.py b/cam_slicer/sender/serial_stream.py
index 33b9bbebb12b49879020aab924935aa73186ad6e..2c4134881020a1d855bab509dfd72bd042f96836 100644
--- a/cam_slicer/sender/serial_stream.py
+++ b/cam_slicer/sender/serial_stream.py
@@ -54,25 +54,38 @@ def stream_gcode_live(gcode_path: str | Path, port: str, baud: int) -> None:
             resp = _read_until_ok(ser)
             if resp.lower() != "ok":
                 logging.error("Controller responded with: %s", resp)
                 raise RuntimeError(f"Unexpected response: {resp}")
 
 
 
 def send_gcode_over_serial(gcode_text: str, port: str, baud: int = 115200) -> str:
     """Send raw G-code lines over serial and capture controller responses."""
     if serial is None:
         raise ImportError("pyserial is required for streaming")
 
     lines = [ln.strip() for ln in gcode_text.splitlines() if ln.strip()]
     log_lines = []
     with serial.Serial(port, baud, timeout=1) as ser:
         for cmd in lines:
             ser.write((cmd + "\n").encode())
             logging.info("Sent: %s", cmd)
             resp = ser.readline().decode().strip()
             if resp:
                 logging.info("Received: %s", resp)
             log_lines.append(f"sent: {cmd}")
             if resp:
                 log_lines.append(f"recv: {resp}")
     return "\n".join(log_lines)
+
+
+try:
+    from serial.tools import list_ports  # type: ignore
+except ModuleNotFoundError:  # pragma: no cover - optional dependency
+    list_ports = None  # type: ignore
+
+
+def list_available_ports() -> list[str]:
+    """Return list of available serial port device names."""
+    if list_ports is None:
+        raise ImportError("pyserial is required for listing ports")
+    return [p.device for p in list_ports.comports()]
