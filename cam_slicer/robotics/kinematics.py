import math
import logging
from dataclasses import dataclass

from cam_slicer.logging_config import setup_logging
setup_logging()
# configure logging to a common file. If logging is already configured
# this call has no effect.

# Default axis limits used when clamping motion. Values are in
# mm/s for velocity and mm/s^2 for acceleration.
axis_limits = {
    "X": {"max_velocity": 500.0, "max_acceleration": 1000.0},
    "Y": {"max_velocity": 500.0, "max_acceleration": 1000.0},
    "Z": {"max_velocity": 100.0, "max_acceleration": 500.0},
}


@dataclass
class TransformConfig:
    """Parameters to transform a point.

    Attributes
    ----------
    rotation_deg : float
        Rotation in degrees around the Z axis.
    scale : float
        Uniform scaling factor.
    offset : tuple[float, float, float]
        General offset applied after scaling.
    work_offset : tuple[float, float, float]
        Work coordinate offset e.g. ``G54``.
    tool_offset : tuple[float, float, float]
        Tool length or radius offset.
    shift : tuple[float, float, float]
        Temporary coordinate shift (similar to ``G92``).
    """

    rotation_deg: float = 0.0
    scale: float = 1.0
    offset: tuple = (0.0, 0.0, 0.0)
    work_offset: tuple = (0.0, 0.0, 0.0)
    tool_offset: tuple = (0.0, 0.0, 0.0)
    shift: tuple = (0.0, 0.0, 0.0)


def polar_to_cartesian(r: float, theta_deg: float) -> tuple[float, float]:
    """Convert polar coordinates to cartesian.

    Args:
        r: Radius.
        theta_deg: Angle in degrees.

    Returns:
        Tuple ``(x, y)`` in cartesian space.
    """

    theta = math.radians(theta_deg)
    return r * math.cos(theta), r * math.sin(theta)


def apply_offsets(x: float, y: float, z: float, cfg: TransformConfig) -> tuple[float, float, float]:
    """Apply work, tool and shift offsets to coordinates."""

    x += cfg.work_offset[0] + cfg.tool_offset[0] + cfg.shift[0]
    y += cfg.work_offset[1] + cfg.tool_offset[1] + cfg.shift[1]
    z += cfg.work_offset[2] + cfg.tool_offset[2] + cfg.shift[2]
    return x, y, z


def transform_point(
    x: float,
    y: float,
    z: float,
    config: TransformConfig,
    mode: str = "cartesian",
    prev_point: tuple[float, float, float] | None = None,
    prev_velocity: tuple[float, float, float] | None = None,
    dt: float = 1.0,
    axis_cfg: dict[str, dict[str, float]] | None = None,
    return_velocity: bool = False,
):
    """Apply rotation, scaling and offsets to a point.

    Args:
        x: ``x`` coordinate or radius if ``mode='polar'``.
        y: ``y`` coordinate or angle in degrees if ``mode='polar'``.
        z: ``z`` coordinate.
        config: Transformation parameters.
        mode: ``'cartesian'`` or ``'polar'``.
        prev_point: Previous transformed coordinates for velocity calculation.
        prev_velocity: Previous velocity vector.
        dt: Time delta since the previous point.
        axis_cfg: Optional axis limit dictionary. If ``None`` ``axis_limits`` is
            used.
        return_velocity: When ``True`` return a tuple ``(pos, vel)`` where
            ``vel`` is the clamped velocity.

    Returns:
        Tuple ``(x, y, z)`` after applying the transformation.
    """

    if mode == "polar":
        x, y = polar_to_cartesian(x, y)

    angle = math.radians(config.rotation_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    # Rotate in XY plane
    x_r = x * cos_a - y * sin_a
    y_r = x * sin_a + y * cos_a

    # Apply scaling
    x_s = x_r * config.scale
    y_s = y_r * config.scale
    z_s = z * config.scale

    # Apply offsets
    x_f = x_s + config.offset[0]
    y_f = y_s + config.offset[1]
    z_f = z_s + config.offset[2]

    # Apply work, tool and coordinate shifts
    x_o, y_o, z_o = apply_offsets(x_f, y_f, z_f, config)

    pos = [x_o, y_o, z_o]
    vel = [0.0, 0.0, 0.0]

    if prev_point is not None:
        limits = axis_cfg or axis_limits
        prev_v = prev_velocity or (0.0, 0.0, 0.0)
        for i, ax in enumerate("XYZ"):
            v = (pos[i] - prev_point[i]) / dt
            a = (v - prev_v[i]) / dt
            max_v = limits[ax]["max_velocity"]
            max_a = limits[ax]["max_acceleration"]
            if abs(v) > max_v:
                v = math.copysign(max_v, v)
            if abs(a) > max_a:
                v = prev_v[i] + math.copysign(max_a, a) * dt
            pos[i] = prev_point[i] + v * dt
            vel[i] = v

    if return_velocity:
        return tuple(pos), tuple(vel)

    return tuple(pos)


def apply_pressure_advance(
    feedrate: float, acceleration: float, advance_factor: float
) -> float:
    """Return compensated feedrate using a simple pressure advance model.

    The adjustment is ``acceleration * advance_factor`` which may be
    interpreted as extra feedrate or converted to a delay for firmware that
    cannot vary extrusion speed in real time.

    Parameters
    ----------
    feedrate : float
        Current feedrate in mm/min or mm/s.
    acceleration : float
        Acceleration applied to the movement.
    advance_factor : float
        Multiplier controlling the amount of compensation.

    Returns
    -------
    float
        Modified feedrate with pressure advance compensation.
    """

    if advance_factor <= 0:
        return feedrate

    advance = acceleration * advance_factor
    return feedrate + advance


def calculate_junction_velocity(
    prev_point: tuple[float, float, float],
    next_point: tuple[float, float, float],
    max_acceleration: float,
    junction_deviation: float,
) -> float:
    """Return limited feedrate for smoother corners.

    This computes the maximum velocity allowed at the junction of two
    consecutive segments using a simple junction deviation model.  The
    vectors ``prev_point`` and ``next_point`` represent the directions of the
    segments before and after the corner.

    Parameters
    ----------
    prev_point : tuple
        Direction vector of the previous move.
    next_point : tuple
        Direction vector of the next move.
    max_acceleration : float
        Maximum available acceleration.
    junction_deviation : float
        Allowed deviation from the programmed path.

    Returns
    -------
    float
        Maximum feedrate that maintains smooth cornering. ``inf`` if the
        path is straight.
    """

    # Vector magnitudes
    mag1 = math.sqrt(sum(c * c for c in prev_point))
    mag2 = math.sqrt(sum(c * c for c in next_point))
    if mag1 == 0 or mag2 == 0:
        return 0.0

    # Clamp cosine to avoid numerical errors
    dot = sum(p * n for p, n in zip(prev_point, next_point))
    cos_theta = max(-1.0, min(1.0, dot / (mag1 * mag2)))

    sin_half = math.sqrt(0.5 * (1.0 - cos_theta))
    if sin_half < 1e-6:
        return float("inf")

    v = math.sqrt(max_acceleration * junction_deviation * sin_half / (1.0 - sin_half))

    logging.info(
        "Junction velocity between %s and %s limited to %.3f",
        prev_point,
        next_point,
        v,
    )
    return v


def _zv_kernel(frequency: float, damping: float, dt: float) -> list[float]:
    """Return discrete ZV input shaping kernel.

    Parameters
    ----------
    frequency : float
        Natural frequency of the system in Hz.
    damping : float
        Damping ratio ``0 <= damping < 1``.
    dt : float
        Time step between motion points.

    Returns
    -------
    list[float]
        Kernel coefficients separated by the appropriate delay.
    """

    if damping >= 1.0:
        damping = 0.999

    wn = 2 * math.pi * frequency
    k = math.exp(-damping * math.pi / math.sqrt(1 - damping**2))
    a1 = 1 / (1 + k)
    a2 = k / (1 + k)
    delay = math.pi / (wn * math.sqrt(1 - damping**2))
    delay_steps = max(1, int(round(delay / dt)))
    kernel = [a1] + [0.0] * (delay_steps - 1) + [a2]
    return kernel


def apply_input_shaping(
    points: list[tuple[float, float, float]],
    frequency: float = 35.0,
    damping: float = 0.1,
) -> list[tuple[float, float, float]]:
    """Filter motion points using a Zero Vibration (ZV) kernel.

    Each coordinate is processed with a simple convolution where the
    delay between kernel impulses is ``1 / frequency``.  ``damping`` controls
    the magnitude of the second impulse.  The output contains the same number
    of points as the input with transformed ``(x, y, z)`` values.

    Parameters
    ----------
    points : list of tuple
        Motion points ``[(x, y, z), ...]``.
    frequency : float, optional
        Shaping frequency in Hz.  Determines the delay between impulses.
    damping : float, optional
        Damping ratio ``0 <= damping < 1``.

    Returns
    -------
    list of tuple
        Shaped point list.
    """

    if not points:
        return []

    if damping >= 1.0:
        damping = 0.999

    k = math.exp(-damping * math.pi / math.sqrt(1 - damping**2))
    a1 = 1 / (1 + k)
    a2 = k / (1 + k)

    delay_steps = max(1, int(round(1.0 / frequency)))

    shaped: list[tuple[float, float, float]] = []
    for i in range(len(points)):
        sx = sy = sz = 0.0
        px, py, pz = points[i]
        sx += a1 * px
        sy += a1 * py
        sz += a1 * pz

        j = i - delay_steps
        if j >= 0:
            px2, py2, pz2 = points[j]
            sx += a2 * px2
            sy += a2 * py2
            sz += a2 * pz2

        shaped.append((sx, sy, sz))

    logging.info(
        "Input shaping applied to %d points with frequency %.2f Hz and damping %.3f",
        len(points),
        frequency,
        damping,
    )

    return shaped


def plan_feedrate_with_lookahead(
    toolpath: list[tuple[float, float, float]],
    max_feedrate: float,
    max_acceleration: float,
    junction_deviation: float,
    queue_size: int = 3,
) -> list[float]:
    """Calculate feedrates ahead of time using a lookahead queue.

    Each point's feedrate is limited by the smallest junction velocity found
    in the next ``queue_size`` points. The queue is rebuilt for every step so
    short paths work as well. Returned list has the same length as the
    ``toolpath``.
    """

    n = len(toolpath)
    if n == 0:
        return []

    feedrates = [max_feedrate for _ in range(n)]

    for i in range(n - 2):
        lookahead_queue = toolpath[i : i + queue_size + 1]
        min_jv = max_feedrate
        for j in range(len(lookahead_queue) - 2):
            prev_vec = tuple(
                lookahead_queue[j + 1][k] - lookahead_queue[j][k] for k in range(3)
            )
            next_vec = tuple(
                lookahead_queue[j + 2][k] - lookahead_queue[j + 1][k] for k in range(3)
            )
            v = calculate_junction_velocity(
                prev_vec, next_vec, max_acceleration, junction_deviation
            )
            min_jv = min(min_jv, v)

        feedrates[i + 1] = min(feedrates[i + 1], min_jv)

    logging.info(
        "Lookahead feedrate planning generated %d values using queue size %d",
        len(feedrates),
        queue_size,
    )
    return feedrates
