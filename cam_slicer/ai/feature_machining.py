import logging
from typing import List, Dict, Tuple

try:
    import trimesh  # type: ignore
    from shapely.geometry import Polygon
except Exception:  # pragma: no cover - optional dependency
    trimesh = None
    Polygon = None

from cam_slicer.logging_config import setup_logging
from cam_slicer.shapes import generate_adaptive_path, generate_spiral_helix

setup_logging()

Feature = Dict[str, object]


def detect_mesh_features(mesh: 'trimesh.Trimesh') -> List[Feature]:
    """Detect basic features such as pockets, holes and slots.

    Uses a mid-plane cross section. Holes are derived from interior rings and
    classified by aspect ratio.
    """
    if trimesh is None or Polygon is None:
        raise ImportError("trimesh and shapely are required for feature detection")

    z_levels = sorted(set(round(v[2], 4) for v in mesh.vertices))
    if len(z_levels) < 2:
        return []
    mid_z = (z_levels[0] + z_levels[-1]) / 2
    section = mesh.section(plane_origin=[0, 0, mid_z], plane_normal=[0, 0, 1])
    if section is None:
        return []
    planar = section.to_planar()
    features: List[Feature] = []
    for poly in planar.polygons_full:
        ext_coords = [(float(x), float(y)) for x, y in poly.exterior.coords]
        if Polygon(poly.exterior).area <= 0:
            continue
        features.append({"type": "pocket", "polygon": ext_coords})
        for ring in poly.interiors:
            ring_poly = Polygon(ring)
            if ring_poly.area == 0:
                continue
            ratio = (ring_poly.length ** 2) / ring_poly.area
            if ratio < 20:
                ftype = "hole"
            else:
                ftype = "slot"
            features.append({"type": ftype, "polygon": [(float(x), float(y)) for x, y in ring.coords]})
    logging.info("Detected %d features", len(features))
    return features


def generate_feature_toolpaths(mesh: 'trimesh.Trimesh', depth: float, tool_diameter: float = 1.0) -> List[List[Tuple[float, float, float]]]:
    """Generate toolpaths from detected features.

    Parameters
    ----------
    mesh : trimesh.Trimesh
        Input mesh to analyze.
    depth : float
        Target machining depth.
    tool_diameter : float, optional
        Cutter diameter to estimate stepover and pitch.

    Returns
    -------
    list of list
        Toolpath segments for machining the detected features.
    """
    if trimesh is None:
        raise ImportError("trimesh is required")

    features = detect_mesh_features(mesh)
    toolpaths: List[List[Tuple[float, float, float]]] = []
    for feat in features:
        poly = feat["polygon"]  # type: ignore[index]
        if feat["type"] in {"pocket", "slot"}:
            segs = generate_adaptive_path(poly, depth=depth, stepdown=tool_diameter, stepover=tool_diameter * 0.5)
            toolpaths.extend(segs)
        elif feat["type"] == "hole":
            poly_shape = Polygon(poly)
            radius = poly_shape.area / poly_shape.length if poly_shape.length else tool_diameter / 2
            c = poly_shape.centroid
            path = generate_spiral_helix((c.x, c.y), radius, depth=depth, pitch=tool_diameter)
            toolpaths.append(path)
        else:  # boss or unknown
            segs = generate_adaptive_path(poly, depth=depth, stepdown=tool_diameter, stepover=tool_diameter * 0.5)
            toolpaths.extend(segs)
    logging.info("Generated %d toolpath segments", len(toolpaths))
    return toolpaths
