import logging
from typing import Dict
from cam_slicer.logging_config import setup_logging
setup_logging()

try:
    import trimesh
except Exception:  # pragma: no cover - optional dependency
    trimesh = None


def analyze_stl(file_path: str) -> Dict[str, float | int | list]:
    """Analyze STL mesh and return basic metrics and warnings."""
    if trimesh is None:  # pragma: no cover - dependency missing
        logging.error("trimesh library not available")
        return {}

    try:
        mesh = trimesh.load(file_path, force="mesh")
    except Exception as exc:  # pragma: no cover - file error
        logging.error("Failed to read STL %s: %s", file_path, exc)
        return {}

    polycount = len(mesh.faces)
    min_edge = float(mesh.edges_unique_length.min()) if mesh.edges_unique_length.size else 0.0
    max_edge = float(mesh.edges_unique_length.max()) if mesh.edges_unique_length.size else 0.0
    bbox = mesh.bounding_box.extents

    warnings: list[str] = []
    if polycount > 100000:
        warnings.append("High polygon count may slow machining")
    if min_edge < 0.1:
        warnings.append("Very small features detected")
    if not mesh.is_watertight:
        warnings.append("Model has holes or is not watertight")
    if len(mesh.split()) > 1:
        warnings.append("Model contains disconnected parts")

    report = {
        "polygons": polycount,
        "min_edge": round(min_edge, 4),
        "max_edge": round(max_edge, 4),
        "bbox": [round(float(v), 3) for v in bbox],
        "warnings": warnings,
    }
    logging.info("Analyzed STL %s: %s", file_path, report)
    return report


def mesh_simplify(file_path: str, target_polycount: int, out_path: str | None = None) -> str:
    """Simplify mesh using quadratic decimation and optionally save to file."""
    if trimesh is None:  # pragma: no cover
        logging.error("trimesh library not available")
        return file_path

    mesh = trimesh.load(file_path, force="mesh")
    simplified = mesh.simplify_quadratic_decimation(target_polycount)
    output = out_path or file_path
    simplified.export(output)
    logging.info("Simplified mesh %s to %d faces", output, len(simplified.faces))
    return output


def repair_mesh(file_path: str, out_path: str | None = None) -> Dict[str, int]:
    """Fix common mesh issues like holes or flipped normals.

    Parameters
    ----------
    file_path : str
        Path to the mesh file (STL or OBJ).
    out_path : str | None, optional
        Where to save the repaired mesh. If ``None``, the original file is
        overwritten.

    Returns
    -------
    Dict[str, int]
        Report with counts of fixed issues.
    """
    if trimesh is None:  # pragma: no cover - optional dependency
        logging.error("trimesh library not available")
        return {}

    mesh = trimesh.load(file_path, force="mesh")

    report = {
        "holes_filled": 0,
        "normals_fixed": 0,
        "nonmanifold_fixed": 0,
    }

    if not mesh.is_watertight:
        before = mesh.euler_number
        mesh.fill_holes()
        report["holes_filled"] = before - mesh.euler_number

    if not mesh.face_normals.any():
        mesh.rezero()
        mesh.fix_normals()
        report["normals_fixed"] = 1

    if mesh.edges_nonmanifold:
        cnt = len(mesh.edges_nonmanifold)
        mesh.remove_unreferenced_vertices()
        mesh.remove_degenerate_faces()
        mesh.remove_duplicate_faces()
        report["nonmanifold_fixed"] = cnt

    output = out_path or file_path
    mesh.export(output)
    logging.info("Repaired mesh saved to %s: %s", output, report)
    return report


def import_mesh_parametric(filepath: str, operations: list | None = None):
    """Import a mesh and apply simple parametric operations.

    Supported operations: ``{"scale": float, "union": path, "difference": path}``.

    Parameters
    ----------
    filepath : str
        Path to the base mesh (STL or OBJ).
    operations : list | None, optional
        Sequence of operations, e.g., ``[{"scale": 2.0}, {"union": "jig.stl"}]``.

    Returns
    -------
    trimesh.Trimesh
        Resulting mesh after applying operations.
    """
    if trimesh is None:  # pragma: no cover - optional dependency
        logging.error("trimesh library not available")
        return None

    base = trimesh.load(filepath, force="mesh")

    for op in operations or []:
        if "scale" in op:
            base.apply_scale(float(op["scale"]))
        elif "union" in op:
            try:
                other = trimesh.load(op["union"], force="mesh")
                base = trimesh.boolean.union([base, other])  # type: ignore
            except Exception as exc:  # pragma: no cover - boolean may fail
                logging.error("Boolean union failed: %s", exc)
        elif "difference" in op:
            try:
                other = trimesh.load(op["difference"], force="mesh")
                base = trimesh.boolean.difference([base, other])  # type: ignore
            except Exception as exc:  # pragma: no cover
                logging.error("Boolean difference failed: %s", exc)

    logging.info("Imported mesh %s with %d operations", filepath, len(operations or []))
    return base
