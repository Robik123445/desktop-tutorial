import logging
from dataclasses import dataclass
from typing import List, Tuple



from cam_slicer.logging_config import setup_logging
setup_logging()
@dataclass
class Placement:
    """Placed shape description."""

    index: int
    position: Tuple[float, float]
    rotation: int


def bounding_box(points: List[Tuple[float, float]]) -> Tuple[float, float]:
    """Return width and height of an axis-aligned bounding box."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    w = max(xs) - min(xs)
    h = max(ys) - min(ys)
    return w, h


def nest_shapes(
    shapes: List[List[Tuple[float, float]]],
    sheet_width: float,
    sheet_height: float,
    margin: float = 0.0,
    allow_rotation: bool = True,
) -> List[Placement]:
    """Pack closed shapes onto a sheet using a simple first-fit algorithm.

    Shapes are sorted by area and placed left-to-right in rows. Each shape may
    be rotated by 90 degrees if ``allow_rotation`` is True. The algorithm is
    greedy â it always picks the orientation that leaves the smallest leftover
    width in the current row. If the shape no longer fits, a new row is started.

    Parameters
    ----------
    shapes : list
        List of closed polygons as ``[(x, y), ...]`` point lists.
    sheet_width : float
        Width of the rectangular sheet.
    sheet_height : float
        Height of the rectangular sheet.
    margin : float, optional
        Space between shapes and sheet edges.
    allow_rotation : bool, optional
        Whether 90Â° rotation is allowed for better fit.

    Returns
    -------
    list of Placement
        Placement data ``(index, (x, y), rotation)`` for each shape.
    """

    dims = [bounding_box(pts) for pts in shapes]
    order = sorted(range(len(shapes)), key=lambda i: dims[i][0] * dims[i][1], reverse=True)

    placements: List[Placement] = []
    x = y = margin
    row_height = 0.0

    for idx in order:
        w, h = dims[idx]
        # Determine the best orientation for the current row
        choices = [(w, h, 0)]
        if allow_rotation and w != h:
            choices.append((h, w, 90))

        best = None
        best_leftover = None
        for cw, ch, rot in choices:
            if x + cw + margin <= sheet_width and y + max(ch, row_height) + margin <= sheet_height:
                leftover = sheet_width - (x + cw + margin)
                if best is None or leftover < best_leftover:
                    best = (cw, ch, rot)
                    best_leftover = leftover

        # Start a new row if nothing fits in the current one
        if best is None:
            y += row_height + margin
            x = margin
            row_height = 0.0
            for cw, ch, rot in choices:
                if x + cw + margin <= sheet_width and y + ch + margin <= sheet_height:
                    best = (cw, ch, rot)
                    break
            if best is None:
                raise ValueError("Shapes do not fit on sheet")

        bw, bh, rotation = best
        placements.append(Placement(idx, (x, y), rotation))
        logging.info("Placed shape %d at (%.2f, %.2f) rot=%d", idx, x, y, rotation)

        x += bw + margin
        row_height = max(row_height, bh)

    return placements
