import logging
import math
from typing import List, Tuple

from cam_slicer.logging_config import setup_logging
setup_logging()
try:  # optional dependencies
    import ezdxf  # type: ignore
except Exception:  # pragma: no cover - missing optional dependency
    ezdxf = None
try:
    import trimesh
except Exception:  # pragma: no cover - missing optional dependency
    trimesh = None


def analyze_dxf(file_path: str) -> dict:
    """Return count of entities and bounding box of a DXF file."""
    if ezdxf is None:  # pragma: no cover - optional dependency
        logging.error("ezdxf library not available")
        return {}

    try:
        doc = ezdxf.readfile(file_path)
    except Exception as exc:  # pragma: no cover
        logging.error("Failed to read DXF %s: %s", file_path, exc)
        return {}

    msp = doc.modelspace()
    count = sum(1 for _ in msp)
    bbox = msp.bbox()
    size = (
        bbox.extmax.x - bbox.extmin.x,
        bbox.extmax.y - bbox.extmin.y,
    )
    report = {"entities": count, "size": [float(s) for s in size]}
    logging.info("Analyzed DXF %s: %s", file_path, report)
    return report



def import_dwg(
    filepath: str,
    *,
    scale: float = 1.0,
    rotation_deg: float = 0.0,
    units: str = "mm",
) -> List[Tuple[float, float]]:
    """Load 2D shapes from a DWG or DXF file and return list of XY tuples."""

    if ezdxf is None:  # pragma: no cover - optional dependency
        logging.error("ezdxf library not available")
        return []

    unit_scale = 25.4 if units.lower() in {"inch", "in"} else 1.0
    angle = math.radians(rotation_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    try:
        doc = ezdxf.readfile(filepath)
    except Exception as exc:  # pragma: no cover - file may not exist
        logging.error("Failed to read DWG %s: %s", filepath, exc)
        return []

    msp = doc.modelspace()
    points: List[Tuple[float, float]] = []
    for entity in msp:
        if entity.dxftype() == "LINE":
            start = entity.dxf.start
            end = entity.dxf.end
            points.extend([(start.x, start.y), (end.x, end.y)])
        elif entity.dxftype() in {"LWPOLYLINE", "POLYLINE"}:
            for p in entity.get_points():
                points.append((p[0], p[1]))

    transformed: List[Tuple[float, float]] = []
    for x, y in points:
        x *= unit_scale * scale
        y *= unit_scale * scale
        xr = x * cos_a - y * sin_a
        yr = x * sin_a + y * cos_a
        transformed.append((xr, yr))

    logging.info("Imported %d points from %s", len(transformed), filepath)
    return transformed


def import_obj(
    filepath: str,
    *,
    scale: float = 1.0,
    rotation_deg: float = 0.0,
) -> List[Tuple[float, float, float]]:
    """Load mesh vertices from an OBJ file."""

    if trimesh is None:  # pragma: no cover - optional dependency
        logging.error("trimesh library not available")
        return []

    angle = math.radians(rotation_deg)
    rot = trimesh.transformations.rotation_matrix(angle, (0, 0, 1))
    try:
        mesh = trimesh.load(filepath, force="mesh")
    except Exception as exc:  # pragma: no cover
        logging.error("Failed to read OBJ %s: %s", filepath, exc)
        return []

    mesh.apply_scale(scale)
    if rotation_deg:
        mesh.apply_transform(rot)

    vertices = [tuple(map(float, v)) for v in mesh.vertices]
    logging.info("Imported OBJ with %d vertices", len(vertices))
    return vertices


def import_stl(
    filepath: str,
    *,
    scale: float = 1.0,
    rotation_deg: float = 0.0,
) -> List[Tuple[float, float, float]]:
    """Load vertices from an STL file."""

    if trimesh is None:  # pragma: no cover - optional dependency
        logging.error("trimesh library not available")
        return []

    angle = math.radians(rotation_deg)
    rot = trimesh.transformations.rotation_matrix(angle, (0, 0, 1))
    try:
        mesh = trimesh.load(filepath, force="mesh")
    except Exception as exc:  # pragma: no cover
        logging.error("Failed to read STL %s: %s", filepath, exc)
        return []

    mesh.apply_scale(scale)
    if rotation_deg:
        mesh.apply_transform(rot)

    vertices = [tuple(map(float, v)) for v in mesh.vertices]
    logging.info("Imported STL with %d vertices", len(vertices))
    return vertices
