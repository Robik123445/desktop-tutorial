import logging
from pathlib import Path
from typing import Iterable, Tuple, List
from cam_slicer.utils import ZMap
from .preview import parse_gcode



from cam_slicer.logging_config import setup_logging
setup_logging()
def _load_heightmap(data: Iterable[Tuple[float, float, float]] | str) -> List[Tuple[float, float, float]]:
    """Return list of points from iterable or file path."""
    if isinstance(data, (str, Path)):
        zmap = ZMap.load(str(data))
        return zmap.points
    return list(data)


def render_surface_and_toolpath(
    heightmap_path: str,
    gcode_path: str,
    *,
    alpha: float = 0.6,
    show: bool = False,
) -> "matplotlib.figure.Figure | list":
    """Overlay scanned surface and toolpath in a 3D plot.

    Parameters
    ----------
    heightmap_path : str
        Path to a JSON or CSV heightmap describing ``(x, y, z)`` points.
    gcode_path : str
        G-code file to overlay.
    alpha : float, optional
        Transparency for the surface mesh (``0`` = transparent). Default ``0.6``.
    show : bool, optional
        Show the plot with ``plt.show`` when True.

    Returns
    -------
    matplotlib.figure.Figure
        Figure instance with the rendered overlay. When matplotlib is not
        available the raw points and toolpath are returned instead.
    """

    points = _load_heightmap(heightmap_path)
    lines = Path(gcode_path).read_text().splitlines()
    toolpath = parse_gcode(lines)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        from matplotlib.tri import Triangulation
    except Exception as exc:  # pragma: no cover
        logging.error("Matplotlib not available: %s", exc)
        return points, toolpath

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    triang = Triangulation(xs, ys)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_trisurf(triang, zs, cmap="viridis", alpha=alpha)

    tp_x = [p[0] for p in toolpath]
    tp_y = [p[1] for p in toolpath]
    tp_z = [p[2] for p in toolpath]
    ax.plot3D(tp_x, tp_y, tp_z, color="red")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    logging.info("Rendered surface overlay with %d surface points and %d toolpath points", len(points), len(toolpath))

    if show:
        plt.show()

    return fig


def _load_shape_points(path: str) -> List[Tuple[float, float, float]]:
    """Return vertices from an STL or SVG file."""
    import numpy as np
    import trimesh

    ext = Path(path).suffix.lower()
    if ext == ".svg":
        ent = trimesh.load_path(path)
        points_2d = np.vstack([e.discrete(50) for e in ent.entities])
        points = np.column_stack((points_2d, np.zeros(len(points_2d))))
    else:
        mesh = trimesh.load_mesh(path)
        if hasattr(mesh, "vertices"):
            points = mesh.vertices
        else:
            raise ValueError("Unsupported shape file")
    return [(float(x), float(y), float(z)) for x, y, z in points]


def _best_fit_transform(
    a: List[Tuple[float, float, float]], b: List[Tuple[float, float, float]]
) -> Tuple[Tuple[float, float, float, float, float, float, float], List[Tuple[float, float, float]]]:
    """Simple ICP algorithm returning transform and transformed points."""
    import numpy as np

    src = np.array(a)
    dst = np.array(b)
    prev_error = float("inf")
    transform = np.eye(4)

    for _ in range(50):
        dists = ((src[:, None, :] - dst[None, :, :]) ** 2).sum(axis=2)
        indices = dists.argmin(axis=1)
        target = dst[indices]

        src_mean = src.mean(axis=0)
        dst_mean = target.mean(axis=0)

        src_centered = src - src_mean
        dst_centered = target - dst_mean

        H = src_centered.T @ dst_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T
        t = dst_mean - R @ src_mean

        src = (R @ src.T).T + t

        error = np.mean(np.linalg.norm(src - target, axis=1))
        if abs(prev_error - error) < 1e-5:
            break
        prev_error = error

        update = np.eye(4)
        update[:3, :3] = R
        update[:3, 3] = t
        transform = update @ transform

    aligned = src.tolist()
    tf_tuple = tuple(transform.flatten())
    return tf_tuple, aligned


def align_toolpath_to_surface(
    shape_path: str,
    surface_data: Iterable[Tuple[float, float, float]] | str,
) -> Tuple[List[Tuple[float, float, float]], List[Tuple[float, float, float]]]:
    """Align toolpath shape to scanned surface using ICP."""

    surface_points = _load_heightmap(surface_data)
    shape_points = _load_shape_points(shape_path)

    _, aligned = _best_fit_transform(shape_points, surface_points)

    logging.info(
        "Aligned %d toolpath points to %d surface points",
        len(shape_points),
        len(surface_points),
    )

    return aligned, surface_points


def simulate_motion_over_surface(
    heightmap_path: str,
    gcode_path: str,
    *,
    step_time: float = 0.1,
    show: bool = False,
) -> "matplotlib.animation.FuncAnimation | tuple":
    """Animate tool motion over the scanned surface.

    Parameters
    ----------
    heightmap_path : str
        Path to the heightmap file (JSON or CSV).
    gcode_path : str
        Path to G-code file describing the toolpath.
    step_time : float, optional
        Time in seconds between frames. Default ``0.1``.
    show : bool, optional
        Display the animation using ``plt.show()``.

    Returns
    -------
    matplotlib.animation.FuncAnimation or tuple
        Animation object if matplotlib is available. Otherwise the raw
        points and toolpath are returned.
    """

    points = _load_heightmap(heightmap_path)
    zmap = ZMap(points)
    lines = Path(gcode_path).read_text().splitlines()
    toolpath = parse_gcode(lines)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        from matplotlib.tri import Triangulation
        from matplotlib.animation import FuncAnimation
    except Exception as exc:  # pragma: no cover
        logging.error("Matplotlib not available: %s", exc)
        return points, toolpath

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    triang = Triangulation(xs, ys)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_trisurf(triang, zs, cmap="viridis", alpha=0.6)

    tp_x = [p[0] for p in toolpath]
    tp_y = [p[1] for p in toolpath]
    tp_z = [p[2] + zmap.get_offset(p[0], p[1]) for p in toolpath]
    ax.plot3D(tp_x, tp_y, tp_z, color="gray", alpha=0.5)

    point, = ax.plot([tp_x[0]], [tp_y[0]], [tp_z[0]], "ro")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    paused = False
    step = False

    def on_key(event):
        nonlocal paused, step
        if event.key == " ":
            paused = not paused
        elif event.key in ("n", "right"):
            step = True

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(frame):
        nonlocal paused, step
        if paused and not step:
            return point,
        if step:
            step = False
            paused = True
        x, y, z = toolpath[frame]
        z += zmap.get_offset(x, y)
        point.set_data([x], [y])
        point.set_3d_properties([z])
        return point,

    ani = FuncAnimation(
        fig,
        update,
        frames=range(len(toolpath)),
        interval=step_time * 1000,
        blit=True,
        repeat=False,
    )

    logging.info(
        "Simulated motion with %d frames over %d surface points",
        len(toolpath),
        len(points),
    )

    if show:
        plt.show()

    return ani


def analyze_toolpath_vs_surface(
    heightmap_data: dict,
    gcode_path: str,
    *,
    clearance: float = 0.1,
    show: bool = False,
) -> list:
    """Return points where the tool collides or loses contact with the surface.

    Parameters
    ----------
    heightmap_data : dict
        Heightmap dictionary ``{"points": [{"x": x, "y": y, "z": z}, ...]}``.
    gcode_path : str
        Path to the G-code file.
    clearance : float, optional
        Extra gap above the surface considered "no contact". Default ``0.1``.
    show : bool, optional
        Display the result using ``plt.show()`` when ``True``.

    Returns
    -------
    list
        List of dictionaries describing every collision or noâcontact point.
    """

    raw_pts = heightmap_data.get("points", [])
    points = [(float(p["x"]), float(p["y"]), float(p["z"])) for p in raw_pts]
    zmap = ZMap(points)
    lines = Path(gcode_path).read_text().splitlines()

    path_points: List[Tuple[float, float, float]] = []
    x = y = z = 0.0
    for line in lines:
        line = line.strip().split(";")[0]
        if not line or not line.startswith("G1"):
            continue
        if "X" in line:
            try:
                x = float(line.split("X")[1].split()[0])
            except ValueError:
                pass
        if "Y" in line:
            try:
                y = float(line.split("Y")[1].split()[0])
            except ValueError:
                pass
        if "Z" in line:
            try:
                z = float(line.split("Z")[1].split()[0])
            except ValueError:
                pass
        path_points.append((x, y, z))

    collisions: List[Tuple[float, float, float]] = []
    high: List[Tuple[float, float, float]] = []
    ok_points: List[Tuple[float, float, float]] = []
    problems = []

    for x, y, z in path_points:
        surf_z = zmap.get_offset(x, y)
        diff = z - surf_z
        if diff < 0:
            collisions.append((x, y, z))
            problems.append({"type": "COLLISION", "x": x, "y": y, "diff": diff})
        elif diff > clearance:
            high.append((x, y, z))
            problems.append({"type": "NO_CONTACT", "x": x, "y": y, "diff": diff})
        else:
            ok_points.append((x, y, z))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except Exception as exc:  # pragma: no cover
        logging.error("Matplotlib not available: %s", exc)
        return problems

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    if collisions:
        cx = [p[0] for p in collisions]
        cy = [p[1] for p in collisions]
        cz = [p[2] for p in collisions]
        ax.scatter(cx, cy, cz, color="red", label="collision")

    if high:
        hx = [p[0] for p in high]
        hy = [p[1] for p in high]
        hz = [p[2] for p in high]
        ax.scatter(hx, hy, hz, color="blue", label="no contact")

    if ok_points:
        ox = [p[0] for p in ok_points]
        oy = [p[1] for p in ok_points]
        oz = [p[2] for p in ok_points]
        ax.scatter(ox, oy, oz, color="green", label="ok")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend(loc="best")

    logging.info(
        "Toolpath analysis: %d collisions, %d high points", len(collisions), len(high)
    )

    if show:
        plt.show()

    return problems
