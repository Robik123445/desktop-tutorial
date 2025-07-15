+"""Simple G-code toolpath parser and simulator."""
+
+from __future__ import annotations
+
+import base64
+import logging
+from io import BytesIO
+from pathlib import Path
+from typing import List, Tuple, Sequence, Optional
+
+import matplotlib.pyplot as plt
+
+from .logging_config import setup_logging
+
+# configure logging and ensure log.txt is used
+setup_logging()
+logger = logging.getLogger(__name__)
+_log_path = Path("logs/log.txt")
+if not any(isinstance(h, logging.FileHandler) and getattr(h, "baseFilename", "") == str(_log_path) for h in logging.getLogger().handlers):
+    _log_path.parent.mkdir(exist_ok=True)
+    _h = logging.FileHandler(_log_path)
+    _h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
+    logging.getLogger().addHandler(_h)
+
+
+def parse_toolpath(gcode_text: str) -> List[Tuple[float, float]]:
+    """Return XY points from G0/G1 lines.
+
+    Lines without coordinates are ignored. Current position is updated when
+    X or Y is found. Only movements using G0 or G1 are considered.
+    """
+    x = y = 0.0
+    points: List[Tuple[float, float]] = []
+    for line in gcode_text.splitlines():
+        line = line.strip()
+        if not line or line.startswith(";"):
+            continue
+        tokens = line.split()
+        if not tokens:
+            continue
+        cmd = tokens[0].upper()
+        if cmd not in {"G0", "G1"}:
+            continue
+        for t in tokens[1:]:
+            if len(t) < 2:
+                continue
+            prefix, num = t[0], t[1:]
+            try:
+                val = float(num)
+            except ValueError:
+                continue
+            if prefix.upper() == "X":
+                x = val
+            elif prefix.upper() == "Y":
+                y = val
+        points.append((x, y))
+    logger.info("Parsed %d toolpath points", len(points))
+    return points
+
+
+def plot_toolpath(points: Sequence[Tuple[float, float]], save_path: Optional[str | Path] = None) -> bytes:
+    """Plot XY toolpath and return PNG bytes.
+
+    When ``save_path`` is provided, the PNG is written to that location.
+    """
+    fig, ax = plt.subplots()
+    if points:
+        xs, ys = zip(*points)
+        ax.plot(xs, ys, "-o")
+    ax.set_xlabel("X")
+    ax.set_ylabel("Y")
+    ax.set_aspect("equal", "box")
+    buf = BytesIO()
+    fig.savefig(buf, format="png")
+    plt.close(fig)
+    data = buf.getvalue()
+    if save_path:
+        Path(save_path).write_bytes(data)
+    logger.info("Generated toolpath plot (%d bytes)", len(data))
+    return data
+
+
+def simulate_toolpath(gcode_text: str, include_plot: bool = False) -> dict:
+    """Parse provided G-code and optionally return a base64 PNG plot."""
+    pts = parse_toolpath(gcode_text)
+    plot_b64: Optional[str] = None
+    if include_plot:
+        png = plot_toolpath(pts)
+        plot_b64 = base64.b64encode(png).decode()
+    result = {"points": pts}
+    if include_plot:
+        result["plot"] = plot_b64
+    logger.info("Simulation result prepared")
+    return result
