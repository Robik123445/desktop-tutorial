import logging
from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)

def emergency_stop() -> None:
    logger.warning("Emergency stop activated")
    print("EMERGENCY STOP triggered!")

DEFAULT_AXIS_RANGE = {
    "X": (-300.0, 300.0),
    "Y": (-300.0, 300.0),
    "Z": (-100.0, 100.0),
}

HARD_CORNER_DEG = 150
MAX_Z_DROP = 10.0

def _angle_between(v1: tuple[float, float, float], v2: tuple[float, float, float]) -> float:
    import math
    dot = sum(a * b for a, b in zip(v1, v2))
    mag1 = math.sqrt(sum(a * a for a in v1))
    mag2 = math.sqrt(sum(b * b for b in v2))
    if mag1 == 0 or mag2 == 0:
        return 0.0
    cos_t = max(-1.0, min(1.0, dot / (mag1 * mag2)))
    return math.degrees(math.acos(cos_t))

def validate_toolpath(
    points: list[tuple[float, float, float]],
    *,
    axis_range: dict[str, tuple[float, float]] | None = None,
    feedrate: float | None = None,
    max_feedrate: float | None = None,
    max_acceleration: float | None = None,
) -> list[str]:
    import math
    axis_range = axis_range or DEFAULT_AXIS_RANGE
    warnings: list[str] = []
    prev_pt = None
    prev_vec = None
    for idx, pt in enumerate(points):
        x, y, z = pt
        for ax, val in zip("XYZ", (x, y, z)):
            rng = axis_range.get(ax)
            if rng and not (rng[0] <= val <= rng[1]):
                raise ValueError(f"Axis {ax} out of range at point {idx}: {val}")

        if prev_pt is not None:
            vec = (x - prev_pt[0], y - prev_pt[1], z - prev_pt[2])
            if prev_vec is not None:
                ang = _angle_between(prev_vec, vec)
                if ang > HARD_CORNER_DEG:
                    warnings.append(f"Hard corner at {idx} ({ang:.1f} deg)")
                    logger.warning("Hard corner at %d: %.1f deg", idx, ang)
                if max_acceleration is not None:
                    dv = abs(math.sqrt(sum(c * c for c in vec)) - math.sqrt(sum(c * c for c in prev_vec)))
                    if dv > max_acceleration:
                        raise ValueError(
                            f"Acceleration {dv:.1f} exceeds limit {max_acceleration} at {idx}"
                        )
            if vec[2] < -MAX_Z_DROP:
                warnings.append(f"Z drop {vec[2]:.1f} at {idx}")
                logger.warning("Dangerous Z drop %.1f at %d", vec[2], idx)
            prev_vec = vec
        else:
            prev_vec = None
        prev_pt = pt

    if feedrate is not None and max_feedrate is not None and feedrate > max_feedrate:
        raise ValueError(f"Feedrate {feedrate} exceeds max {max_feedrate}")

    return warnings
