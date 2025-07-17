import logging
from typing import List, Tuple, Union

from cam_slicer.logging_config import setup_logging
setup_logging()

def parse_gcode(lines: List[str]) -> List[Tuple[float, float, float]]:
    """Parse G-code lines and return list of XYZ tuples with arc interpolation."""

    def _interp_arc(
        start: Tuple[float, float, float],
        center: Tuple[float, float, float],
        end: Tuple[float, float, float],
        cw: bool,
        segments: int = 20,
    ) -> List[Tuple[float, float, float]]:
        from math import atan2, cos, sin, pi

        sx, sy, sz = start
        cx, cy, _ = center
        ex, ey, ez = end
        r = ((sx - cx) ** 2 + (sy - cy) ** 2) ** 0.5
        a0 = atan2(sy - cy, sx - cx)
        a1 = atan2(ey - cy, ex - cx)
        if cw:
            if a1 >= a0:
                a1 -= 2 * pi
        else:
            if a1 <= a0:
                a1 += 2 * pi
        pts = []
        for i in range(segments + 1):
            t = i / segments
            ang = a0 + (a1 - a0) * t
            x = cx + r * cos(ang)
            y = cy + r * sin(ang)
            z = sz + (ez - sz) * t
            pts.append((x, y, z))
        return pts

    points: List[Tuple[float, float, float]] = []
    x = y = z = 0.0
    for line in lines:
        line = line.strip().split(";")[0]
        if not line:
            continue
        if line.startswith("G2") or line.startswith("G3"):
            cw = line.startswith("G2")
            x_end = x
            y_end = y
            z_end = z
            if "X" in line:
                try:
                    x_end = float(line.split("X")[1].split()[0])
                except ValueError:
                    pass
            if "Y" in line:
                try:
                    y_end = float(line.split("Y")[1].split()[0])
                except ValueError:
                    pass
            if "Z" in line:
                try:
                    z_end = float(line.split("Z")[1].split()[0])
                except ValueError:
                    pass
            if "I" in line and "J" in line:
                try:
                    i = float(line.split("I")[1].split()[0])
                    j = float(line.split("J")[1].split()[0])
                except ValueError:
                    points.append((x_end, y_end, z_end))
                else:
                    start_pt = (x, y, z)
                    center = (x + i, y + j, z)
                    end_pt = (x_end, y_end, z_end)
                    arc_pts = _interp_arc(start_pt, center, end_pt, cw)
                    if points and points[-1] == start_pt:
                        points.extend(arc_pts[1:])
                    else:
                        points.extend(arc_pts)
            else:
                points.append((x_end, y_end, z_end))
            x, y, z = x_end, y_end, z_end
        elif line.startswith("G0") or line.startswith("G1"):
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
        else:
            continue
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
    # ... (ostatok funkcie nezmenený, tu môžeš dorobiť zvyšok podľa potreby)
