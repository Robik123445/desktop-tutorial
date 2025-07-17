import csv
import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Sequence

from cam_slicer.logging_config import setup_logging
setup_logging()


@dataclass
class ZMap:
    """Store a grid of Z offsets and provide interpolation."""

    points: List[Tuple[float, float, float]]

    def __post_init__(self) -> None:
        """Precompute lookup tables for fast interpolation."""
        self._lookup = {(x, y): z for x, y, z in self.points}
        self._xs = sorted({x for x, _, _ in self.points})
        self._ys = sorted({y for _, y, _ in self.points})

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
        """Return Z offset for ``(x, y)`` using bilinear interpolation."""
        if not self.points:
            return 0.0

        xs = self._xs
        ys = self._ys

        # find bounding grid coordinates
        x0 = max([v for v in xs if v <= x], default=xs[0])
        x1 = min([v for v in xs if v >= x], default=xs[-1])
        y0 = max([v for v in ys if v <= y], default=ys[0])
        y1 = min([v for v in ys if v >= y], default=ys[-1])

        z00 = self._lookup.get((x0, y0))
        z01 = self._lookup.get((x0, y1))
        z10 = self._lookup.get((x1, y0))
        z11 = self._lookup.get((x1, y1))

        if None in {z00, z01, z10, z11}:
            nearest = min(
                self.points, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2
            )
            return nearest[2]

        if x1 == x0 and y1 == y0:
            return z00
        if x1 == x0:
            # linear interpolation along Y only
            return z00 + (z01 - z00) * ((y - y0) / (y1 - y0))
        if y1 == y0:
            return z00 + (z10 - z00) * ((x - x0) / (x1 - x0))

        t = (x - x0) / (x1 - x0)
        u = (y - y0) / (y1 - y0)
        z0 = z00 * (1 - t) + z10 * t
        z1 = z01 * (1 - t) + z11 * t
        return z0 * (1 - u) + z1 * u


def apply_heightmap_to_gcode(lines: Sequence[str], zmap: "ZMap") -> List[str]:
    """Adjust Z values in G-code using ``zmap`` offsets."""

    adjusted: List[str] = []
    for line in lines:
        parts = line.strip().split()
        if not parts:
            adjusted.append(line)
            continue
        cmd = parts[0]
        if cmd not in {"G0", "G1"}:
            adjusted.append(line)
            continue
        x = y = z = None
        rest: List[str] = []
        for tok in parts[1:]:
            if tok.startswith("X"):
                try:
                    x = float(tok[1:])
                except ValueError:
                    rest.append(tok)
            elif tok.startswith("Y"):
                try:
                    y = float(tok[1:])
                except ValueError:
                    rest.append(tok)
            elif tok.startswith("Z"):
                try:
                    z = float(tok[1:])
                except ValueError:
                    pass
            else:
                rest.append(tok)
        if x is not None and y is not None and z is not None:
            z += zmap.get_offset(x, y)
            new_parts = [cmd, f"X{x:.3f}", f"Y{y:.3f}", f"Z{z:.3f}"] + rest
            line = " ".join(new_parts)
        adjusted.append(line)
    logging.info("Heightmap applied to %d lines", len(adjusted))
    return adjusted
