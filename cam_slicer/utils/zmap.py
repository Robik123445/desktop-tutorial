import csv
import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple


from cam_slicer.logging_config import setup_logging
setup_logging()
@dataclass
class ZMap:
    """Store a grid of Z offsets and provide interpolation."""

    points: List[Tuple[float, float, float]]

    @classmethod
    def load(cls, path: str) -> "ZMap":
        """Load a Z map from CSV or JSON file."""
        ext = Path(path).suffix.lower()
        if ext == ".json":
            return cls._from_json(path)
        return cls._from_csv(path)

    @classmethod
    def _from_csv(cls, path: str) -> "ZMap":
        pts: List[Tuple[float, float, float]] = []
        with open(path, newline="") as f:
            reader = csv.reader(f)
            for row in reader:
                if not row:
                    continue
                x, y, z = map(float, row[:3])
                pts.append((x, y, z))
        logging.info("Loaded %d Z-map points from %s", len(pts), path)
        return cls(pts)

    @classmethod
    def _from_json(cls, path: str) -> "ZMap":
        with open(path) as f:
            data = json.load(f)
        pts: List[Tuple[float, float, float]] = []
        if "points" in data:
            for p in data["points"]:
                pts.append((float(p["x"]), float(p["y"]), float(p["z"])))
        elif "grid" in data:
            grid = data["grid"]
            x0 = float(data.get("x0", 0.0))
            y0 = float(data.get("y0", 0.0))
            x_step = float(data.get("x_step", 1.0))
            y_step = float(data.get("y_step", 1.0))
            for iy, row in enumerate(grid):
                for ix, z in enumerate(row):
                    x = x0 + ix * x_step
                    y = y0 + iy * y_step
                    pts.append((x, y, float(z)))
        else:
            raise ValueError("Unknown JSON Z-map format")
        logging.info("Loaded %d Z-map points from %s", len(pts), path)
        return cls(pts)

    def get_offset(self, x: float, y: float) -> float:
        """Return Z offset for given coordinates using nearest point."""
        if not self.points:
            return 0.0
        nearest = min(self.points, key=lambda p: (p[0]-x)**2 + (p[1]-y)**2)
        return nearest[2]
