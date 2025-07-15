diff --git a//dev/null b/cam_slicer/heightmap.py
index 0000000000000000000000000000000000000000..966476615c0184d352d07add60dde76b09348a18 100644
--- a//dev/null
+++ b/cam_slicer/heightmap.py
@@ -0,0 +1,101 @@
+"""Apply Z heightmap offsets to G-code."""
+
+from __future__ import annotations
+
+import csv
+import json
+import logging
+from pathlib import Path
+from typing import List, Tuple, Optional
+
+from .logging_config import setup_logging
+from .utils import ZMap
+
+setup_logging()
+logger = logging.getLogger(__name__)
+
+_log_path = Path("logs/log.txt")
+if not any(
+    isinstance(h, logging.FileHandler) and getattr(h, "baseFilename", "") == str(_log_path)
+    for h in logging.getLogger().handlers
+):
+    _log_path.parent.mkdir(exist_ok=True)
+    fh = logging.FileHandler(_log_path)
+    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
+    logging.getLogger().addHandler(fh)
+
+
+class HeightMap(ZMap):
+    """Height map loaded from CSV or JSON string."""
+
+    @classmethod
+    def from_text(cls, text: str, fmt: Optional[str] = None) -> "HeightMap":
+        fmt = (fmt or "csv").lower()
+        if fmt not in {"csv", "json"}:
+            fmt = "json" if text.strip().startswith("[") or text.strip().startswith("{") else "csv"
+        if fmt == "json":
+            data = json.loads(text)
+            if isinstance(data, dict) and "points" in data:
+                pts = [(float(p["x"]), float(p["y"]), float(p["z"])) for p in data["points"]]
+            elif isinstance(data, list):
+                pts = [(float(p[0]), float(p[1]), float(p[2])) for p in data]
+            else:
+                raise ValueError("Invalid JSON heightmap")
+            logger.info("Loaded %d heightmap points from JSON text", len(pts))
+            return cls(pts)
+        pts: List[Tuple[float, float, float]] = []
+        for row in csv.reader(text.splitlines()):
+            if not row:
+                continue
+            try:
+                x, y, z = map(float, row[:3])
+            except ValueError:
+                continue
+            pts.append((x, y, z))
+        logger.info("Loaded %d heightmap points from CSV text", len(pts))
+        return cls(pts)
+
+    @classmethod
+    def load(cls, path: str | Path) -> "HeightMap":  # type: ignore[override]
+        path = Path(path)
+        fmt = "json" if path.suffix.lower() == ".json" else "csv"
+        return cls.from_text(path.read_text(), fmt=fmt)
+
+
+def apply_heightmap_to_gcode(gcode: str, heightmap: HeightMap) -> str:
+    """Return G-code string with Z adjusted by the heightmap."""
+    out_lines: List[str] = []
+    cur_x = cur_y = 0.0
+    for line in gcode.splitlines():
+        tokens = line.strip().split()
+        new_tokens: List[str] = []
+        for t in tokens:
+            if len(t) < 2:
+                new_tokens.append(t)
+                continue
+            prefix, num = t[0].upper(), t[1:]
+            if prefix == "X":
+                try:
+                    cur_x = float(num)
+                except ValueError:
+                    pass
+                new_tokens.append(t)
+            elif prefix == "Y":
+                try:
+                    cur_y = float(num)
+                except ValueError:
+                    pass
+                new_tokens.append(t)
+            elif prefix == "Z":
+                try:
+                    val = float(num)
+                except ValueError:
+                    new_tokens.append(t)
+                    continue
+                val += heightmap.get_offset(cur_x, cur_y)
+                new_tokens.append(f"Z{val:.3f}")
+            else:
+                new_tokens.append(t)
+        out_lines.append(" ".join(new_tokens))
+    logger.info("Adjusted %d lines using heightmap", len(out_lines))
+    return "\n".join(out_lines)
