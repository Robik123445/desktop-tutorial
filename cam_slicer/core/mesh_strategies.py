import logging
import math
from typing import List, Tuple, Dict, TYPE_CHECKING
try:
    import trimesh  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    trimesh = None
if TYPE_CHECKING:  # pragma: no cover - for type hints
    import trimesh as tm

from cam_slicer.logging_config import setup_logging
from cam_slicer.shapes import generate_adaptive_path

setup_logging()


def generate_roughing_paths(mesh: 'tm.Trimesh', *, stepdown: float = 1.0, stepover: float = 2.0) -> List[List[Tuple[float, float, float]]]:
    """Generate simple roughing layers for a mesh.

    The mesh bounding box is sliced in ``stepdown`` increments and each layer is
    milled using a zigzag path computed by :func:`generate_adaptive_path`.
    """
    if trimesh is None:
        raise ImportError("trimesh is required for roughing paths")
    min_z = mesh.bounds[0][2]
    max_z = mesh.bounds[1][2]
    boundary = [
        (mesh.bounds[0][0], mesh.bounds[0][1]),
        (mesh.bounds[1][0], mesh.bounds[0][1]),
        (mesh.bounds[1][0], mesh.bounds[1][1]),
        (mesh.bounds[0][0], mesh.bounds[1][1]),
    ]
    toolpaths: List[List[Tuple[float, float, float]]] = []
    current_z = max_z
    while current_z > min_z:
        target = max(current_z - stepdown, min_z)
        layer = generate_adaptive_path(
            boundary,
            depth=target - max_z,
            stepdown=stepdown,
            stepover=stepover,
            mode="zigzag",
        )
        toolpaths.extend(layer)
        logging.info("Roughing layer at %.3f", target)
        current_z -= stepdown
    return toolpaths


def generate_finishing_paths(mesh: 'tm.Trimesh', *, stepover: float = 0.5) -> List[List[Tuple[float, float, float]]]:
    """Generate naive finishing passes following mesh triangles."""
    if trimesh is None:
        raise ImportError("trimesh is required for finishing paths")
    toolpaths: List[List[Tuple[float, float, float]]] = []
    for tri in mesh.triangles:
        tri_path = [tuple(v) for v in tri]
        toolpaths.append(tri_path)
    logging.info("Finishing with %d triangle paths", len(toolpaths))
    return toolpaths


def generate_carving_paths(mesh: 'tm.Trimesh', depth: float, *, stepdown: float = 1.0, stepover: float = 1.0) -> List[List[Tuple[float, float, float]]]:
    """Generate basic carving passes sinking into the mesh."""
    if trimesh is None:
        raise ImportError("trimesh is required for carving paths")
    boundary = [
        (mesh.bounds[0][0], mesh.bounds[0][1]),
        (mesh.bounds[1][0], mesh.bounds[0][1]),
        (mesh.bounds[1][0], mesh.bounds[1][1]),
        (mesh.bounds[0][0], mesh.bounds[1][1]),
    ]
    toolpaths: List[List[Tuple[float, float, float]]] = []
    current_z = 0.0
    while current_z > depth:
        layer = generate_adaptive_path(
            boundary,
            depth=current_z,
            stepdown=stepdown,
            stepover=stepover,
            mode="spiral",
        )
        toolpaths.extend(layer)
        logging.info("Carving layer at %.3f", current_z)
        current_z -= stepdown
    return toolpaths


def generate_zlevel_roughing_paths(
    mesh: 'tm.Trimesh', *, stepdown: float = 1.0
) -> List[List[Tuple[float, float, float]]]:
    """Rough the mesh using constant Z layers.

    Parameters
    ----------
    mesh : :class:`trimesh.Trimesh`
        Mesh to slice into horizontal layers.
    stepdown : float, optional
        Z increment between layers.

    Returns
    -------
    list of list
        Toolpath loops for each Z slice.
    """
    if trimesh is None:
        raise ImportError("trimesh is required for roughing paths")

    min_z, max_z = mesh.bounds[:, 2]
    toolpaths: List[List[Tuple[float, float, float]]] = []
    z = max_z
    while z >= min_z - 1e-6:
        section = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if section is not None:
            polygons = section.to_planar().polygons_full
            for poly in polygons:
                pts = [(float(p[0]), float(p[1]), float(z)) for p in poly]
                toolpaths.append(pts)
            logging.info("Rough Z-plane %.3f with %d loops", z, len(polygons))
        z -= stepdown
    return toolpaths


def generate_zlevel_finishing_paths(
    mesh: 'tm.Trimesh', *, stepdown: float = 0.5, vertical_thresh: float = 0.5
) -> List[List[Tuple[float, float, float]]]:
    """Finish steep regions using constant Z slices.

    Only faces with |normal.z| < ``vertical_thresh`` are considered so the
    passes focus on vertical walls where a constant Z strategy produces the
    best surface finish.
    """
    if trimesh is None:
        raise ImportError("trimesh is required for finishing paths")

    indices = [i for i, n in enumerate(mesh.face_normals) if abs(n[2]) < vertical_thresh]
    if not indices:
        return []
    sub = mesh.submesh([indices], append=True, repair=False)
    min_z, max_z = sub.bounds[:, 2]
    toolpaths: List[List[Tuple[float, float, float]]] = []
    z = max_z
    while z >= min_z - 1e-6:
        section = sub.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if section is not None:
            polygons = section.to_planar().polygons_full
            for poly in polygons:
                pts = [(float(p[0]), float(p[1]), float(z)) for p in poly]
                toolpaths.append(pts)
            logging.info("Finish Z-plane %.3f with %d loops", z, len(polygons))
        z -= stepdown
    return toolpaths


def generate_parallel_finishing_paths(
    mesh: 'tm.Trimesh',
    *,
    stepover: float = 0.5,
    axis: str = "x",
) -> List[List[Tuple[float, float, float]]]:
    """Raster finish the mesh with parallel lines.

    Parameters
    ----------
    mesh : :class:`trimesh.Trimesh`
        Mesh to slice into parallel planes.
    stepover : float, optional
        Distance between adjacent scan lines.
    axis : str, optional
        ``"x"`` to sweep along X (lines vary in Y) or ``"y"`` to sweep along Y.

    Returns
    -------
    list of list
        Ordered polyline segments following the surface profile.
    """
    if trimesh is None:
        raise ImportError("trimesh is required for parallel finishing")

    axis = axis.lower()
    if axis not in {"x", "y"}:
        raise ValueError("axis must be 'x' or 'y'")

    idx = 1 if axis == "x" else 0
    normal = [0.0, 1.0, 0.0] if axis == "x" else [1.0, 0.0, 0.0]
    min_v, max_v = mesh.bounds[:, idx]

    toolpaths: List[List[Tuple[float, float, float]]] = []
    pos = min_v
    while pos <= max_v + 1e-6:
        origin = [0.0, 0.0, 0.0]
        origin[idx] = pos
        section = mesh.section(plane_origin=origin, plane_normal=normal)
        if section is not None:
            planar = section.to_planar()
            for line in getattr(planar, "discrete", []):
                world = planar.to_world(line)
                pts = [(float(p[0]), float(p[1]), float(p[2])) for p in world]
                toolpaths.append(pts)
            logging.info(
                "Parallel finish plane %.3f with %d lines", pos, len(toolpaths)
            )
        pos += stepover
    return toolpaths


def generate_offset_finishing_paths(
    mesh: 'tm.Trimesh',
    *,
    offset: float = 0.1,
    stepover: float = 0.5,
    passes: int = 1,
) -> List[List[Tuple[float, float, float]]]:
    """Finish a mesh using constant offset passes.

    Each pass follows the surface at ``offset + i * stepover`` distance along
    vertex normals, maintaining an even stepover between passes.
    """
    if trimesh is None:
        raise ImportError("trimesh is required for offset finishing")

    toolpaths: List[List[Tuple[float, float, float]]] = []
    for i in range(passes):
        dist = offset + i * stepover
        verts = mesh.vertices + mesh.vertex_normals * dist
        for tri in mesh.faces:
            a, b, c = verts[tri]
            toolpaths.append([
                (float(a[0]), float(a[1]), float(a[2])),
                (float(b[0]), float(b[1]), float(b[2])),
                (float(c[0]), float(c[1]), float(c[2])),
            ])
        logging.info("Offset finish pass %.3f with %d triangles", dist, len(mesh.faces))
    return toolpaths


def generate_high_polish_paths(
    mesh: 'tm.Trimesh',
    *,
    stepover: float = 0.05,
    passes: int = 5,
) -> List[List[Tuple[float, float, float]]]:
    """Create ultra-fine finishing passes for mirror surfaces.

    Parameters
    ----------
    mesh : :class:`trimesh.Trimesh`
        Mesh to finish.
    stepover : float, optional
        Distance between adjacent passes along vertex normals.
    passes : int, optional
        Number of passes to generate starting from the surface.

    Returns
    -------
    list of list
        Finishing paths suitable for high-polish operations.
    """

    if trimesh is None:
        raise ImportError("trimesh is required for high-polish paths")

    toolpaths: List[List[Tuple[float, float, float]]] = []
    for i in range(passes):
        dist = i * stepover
        verts = mesh.vertices + mesh.vertex_normals * dist
        for tri in mesh.faces:
            a, b, c = verts[tri]
            toolpaths.append([
                (float(a[0]), float(a[1]), float(a[2])),
                (float(b[0]), float(b[1]), float(b[2])),
                (float(c[0]), float(c[1]), float(c[2])),
            ])
        logging.info(
            "High-polish pass %.3f with %d triangles", dist, len(mesh.faces)
        )

    return toolpaths


def generate_radial_finishing_paths(
    mesh: 'tm.Trimesh',
    *,
    angle_step: float = 10.0,
) -> List[List[Tuple[float, float, float]]]:
    """Finish the mesh using radial passes.

    Parameters
    ----------
    mesh : :class:`trimesh.Trimesh`
        Mesh to slice with radial planes.
    angle_step : float, optional
        Angular increment in degrees between subsequent radial cuts.

    Returns
    -------
    list of list
        Polyline segments following the surface along radial planes.
    """
    if trimesh is None:
        raise ImportError("trimesh is required for radial finishing")

    center = mesh.bounding_box.centroid
    toolpaths: List[List[Tuple[float, float, float]]] = []
    angle = 0.0
    while angle < 360.0 - 1e-6:
        normal = [
            -math.sin(math.radians(angle)),
            math.cos(math.radians(angle)),
            0.0,
        ]
        section = mesh.section(plane_origin=center, plane_normal=normal)
        if section is not None:
            planar = section.to_planar()
            for line in getattr(planar, "discrete", []):
                world = planar.to_world(line)
                pts = [(float(p[0]), float(p[1]), float(p[2])) for p in world]
                toolpaths.append(pts)
            logging.info(
                "Radial finish angle %.1f with %d lines",
                angle,
                len(getattr(planar, "discrete", [])),
            )
        angle += angle_step
    return toolpaths


def plan_mesh_operations(mesh: 'tm.Trimesh', tools: Dict[str, str]) -> List[Dict]:
    """Plan roughing, finishing and carving passes with tool assignments."""
    if trimesh is None:
        raise ImportError("trimesh is required for mesh operations")
    operations = []
    if "roughing" in tools:
        ops = generate_roughing_paths(mesh)
        operations.append({"operation": "roughing", "tool": tools["roughing"], "toolpaths": ops})
    if "finishing" in tools:
        ops = generate_finishing_paths(mesh)
        operations.append({"operation": "finishing", "tool": tools["finishing"], "toolpaths": ops})
    if "carving" in tools:
        ops = generate_carving_paths(mesh, depth=mesh.bounds[0][2] - 1.0)
        operations.append({"operation": "carving", "tool": tools["carving"], "toolpaths": ops})
    logging.info("Planned %d mesh operations", len(operations))
    return operations


def project_curves_to_surface(
    curves: List[List[Tuple[float, float, float]]],
    mesh: 'tm.Trimesh',
    *,
    offset: float = 0.0,
) -> List[List[Tuple[float, float, float]]]:
    """Project curves onto a mesh surface.

    Parameters
    ----------
    curves : list of list
        Input polylines defined either in 2D or 3D coordinates.
    mesh : :class:`trimesh.Trimesh`
        Target surface onto which the curves are projected.
    offset : float, optional
        Distance to offset along the surface normal after projection.

    Returns
    -------
    list of list
        Projected polylines following the mesh surface.
    """
    if trimesh is None:
        raise ImportError("trimesh is required for projecting curves")

    projected: List[List[Tuple[float, float, float]]] = []
    for seg in curves:
        new_seg: List[Tuple[float, float, float]] = []
        for pt in seg:
            if len(pt) == 2:
                query = (pt[0], pt[1], 0.0)
            else:
                query = pt
            closest, face_idx = mesh.nearest.on_surface([query])
            hit = closest[0]
            normal = mesh.face_normals[face_idx[0]]
            hit = hit + normal * offset
            new_seg.append((float(hit[0]), float(hit[1]), float(hit[2])))
        projected.append(new_seg)
    logging.info("Projected %d curves onto surface", len(projected))
    return projected
