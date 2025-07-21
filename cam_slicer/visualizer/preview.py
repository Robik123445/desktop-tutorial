import logging
from typing import List, Tuple, Union

from cam_slicer.logging_config import setup_logging
setup_logging()


def parse_gcode(lines: List[str]) -> List[Tuple[float, float, float]]:
    """Parse G-code lines and return list of XYZ tuples."""
    points: List[Tuple[float, float, float]] = []
    x = y = z = 0.0
    for line in lines:
        line = line.strip().split(";")[0]  # remove comments
        if not line:
            continue
        if any(cmd in line for cmd in ("G0", "G1", "G2", "G3")):
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
            points.append((x, y, z))
    logging.info("Parsed %d points from gcode", len(points))
    return points


def preview_gcode(
    lines: List[str],
    laser_mode: bool = False,
    show: bool = False,
) -> Union["matplotlib.figure.Figure", List[Tuple[float, float, float]]]:
    """Preview G-code toolpath using matplotlib.

    Parameters
    ----------
    lines : list of str
        G-code commands to visualize.
    laser_mode : bool, optional
        If True, ignore Z depth and draw a flat 2D path.
    show : bool, optional
        Call ``plt.show()`` to display the figure. Default False.

    Returns
    -------
    matplotlib.figure.Figure or list
        Matplotlib figure with the plot. If matplotlib is unavailable, the raw
        points are returned instead.
    """
    pts = parse_gcode(lines)
    try:
        import matplotlib
        matplotlib.use("Agg")  # headless backend
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused
    except Exception as exc:  # pragma: no cover - if mpl missing
        logging.error("Matplotlib not available: %s", exc)
        return pts

    fig = plt.figure()
    if laser_mode:
        ax = fig.add_subplot(111)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, "-o")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
    else:
        ax = fig.add_subplot(111, projection="3d")
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        ax.plot(xs, ys, zs, "-o")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
    logging.info("Generated preview: %s mode", "laser" if laser_mode else "mill")
    if show:
        plt.show()
    return fig


def export_preview_image(
    gcode_path: str,
    out_path: str,
    *,
    laser_mode: bool = False,
    part_id: str | None = None,
) -> None:
    """Save a top-down preview image of a G-code file.

    Parameters
    ----------
    gcode_path : str
        Path to the G-code file.
    out_path : str
        Output image path (.png or .svg).
    laser_mode : bool, optional
        Ignore Z depth when ``True``. Default ``False``.
    part_id : str, optional
        Annotation label, defaults to ``gcode_path`` stem.
    """
    from pathlib import Path

    lines = Path(gcode_path).read_text().splitlines()
    points = parse_gcode(lines)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover
        logging.error("Matplotlib not available: %s", exc)
        return

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    fig, ax = plt.subplots()
    ax.plot(xs, ys, "-k")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)

    if points:
        ax.plot(xs[0], ys[0], "go", label="start")
        ax.plot(xs[-1], ys[-1], "ro", label="end")
        ax.legend(loc="best")

    # simple scale bar
    if xs and ys:
        span = max(max(xs) - min(xs), max(ys) - min(ys))
        bar = span / 5 if span else 1.0
        y_pos = min(ys) - 0.05 * span
        ax.hlines(y_pos, min(xs), min(xs) + bar, colors="r")
        ax.text(min(xs), y_pos - 0.02 * span, f"{bar:.1f} units", color="r")

    part = part_id or Path(gcode_path).stem
    ax.set_title(f"Part: {part}")

    fig.tight_layout()
    plt.savefig(out_path)
    plt.close(fig)
    logging.info("Preview image saved to %s", out_path)


def backplot_gcode(gcode_path: str, *, step_time: float = 0.1, show: bool = False):
    """Animate a G-code file as a 3D backplot.

    Parameters
    ----------
    gcode_path : str
        Path to G-code file to visualize.
    step_time : float, optional
        Time between frames in seconds. Default ``0.1``.
    show : bool, optional
        Display the animation using ``plt.show()`` when ``True``.

    Returns
    -------
    matplotlib.animation.FuncAnimation or list
        Animation object if matplotlib is available, otherwise the parsed
        points list.
    """
    from pathlib import Path

    lines = Path(gcode_path).read_text().splitlines()
    pts = parse_gcode(lines)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        from matplotlib.animation import FuncAnimation
    except Exception as exc:  # pragma: no cover
        logging.error("Matplotlib not available: %s", exc)
        return pts

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    ax.plot(xs, ys, zs, color="gray", alpha=0.3)
    marker, = ax.plot([xs[0]], [ys[0]], [zs[0]], "ro")
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

    def update(frame: int):
        nonlocal paused, step
        if paused and not step:
            return marker,
        if step:
            step = False
            paused = True
        x, y, z = pts[frame]
        marker.set_data([x], [y])
        marker.set_3d_properties([z])
        return marker,

    ani = FuncAnimation(fig, update, frames=range(len(pts)), interval=step_time * 1000, blit=True, repeat=False)
    logging.info("Backplot loaded %d points", len(pts))
    if show:
        plt.show()
    return ani
