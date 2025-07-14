diff --git a/cam_slicer/sender/serial_stream.py b/cam_slicer/sender/serial_stream.py
index 90236dffdacec10619085e7212dfa593226055ba..33b9bbebb12b49879020aab924935aa73186ad6e 100644
--- a/cam_slicer/sender/serial_stream.py
+++ b/cam_slicer/sender/serial_stream.py
@@ -34,25 +34,45 @@ def stream_gcode_live(gcode_path: str | Path, port: str, baud: int) -> None:
     Raises
     ------
     RuntimeError
         When controller response does not contain ``ok``.
     """
 
     if serial is None:
         raise ImportError("pyserial is required for streaming")
 
     path = Path(gcode_path)
     if not path.is_file():
         raise FileNotFoundError(path)
 
     with serial.Serial(port, baud, timeout=1) as ser, path.open("r", encoding="utf-8") as fh:
         for line in fh:
             cmd = line.strip()
             if not cmd:
                 continue
             ser.write((cmd + "\n").encode())
             logging.info("Sent: %s", cmd)
             resp = _read_until_ok(ser)
             if resp.lower() != "ok":
                 logging.error("Controller responded with: %s", resp)
                 raise RuntimeError(f"Unexpected response: {resp}")
 
+
+
+def send_gcode_over_serial(gcode_text: str, port: str, baud: int = 115200) -> str:
+    """Send raw G-code lines over serial and capture controller responses."""
+    if serial is None:
+        raise ImportError("pyserial is required for streaming")
+
+    lines = [ln.strip() for ln in gcode_text.splitlines() if ln.strip()]
+    log_lines = []
+    with serial.Serial(port, baud, timeout=1) as ser:
+        for cmd in lines:
+            ser.write((cmd + "\n").encode())
+            logging.info("Sent: %s", cmd)
+            resp = ser.readline().decode().strip()
+            if resp:
+                logging.info("Received: %s", resp)
+            log_lines.append(f"sent: {cmd}")
+            if resp:
+                log_lines.append(f"recv: {resp}")
+    return "\n".join(log_lines)
