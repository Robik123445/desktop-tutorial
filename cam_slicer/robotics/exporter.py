import logging
from typing import Iterable, Sequence, List, Callable

from cam_slicer.logging_config import setup_logging
from cam_slicer.core.gcode_export import toolpath_to_gcode
from cam_slicer.core.header_footer import ControllerConfig
from cam_slicer.sender.serial_streamer import stream_gcode_to_grbl
import tempfile
import os
from .interface import (
    ArmKinematicProfile,
    format_extended_g1,
    format_move_arm,
)

setup_logging()
logger = logging.getLogger(__name__)


class PostProcessor:
    """Base class for robotic post processors."""

    def format_pose(self, pose: Sequence[float]) -> str:
        return format_extended_g1(*pose)

    def format_joints(self, joints: Sequence[float]) -> str:
        return format_move_arm(list(joints))


def export_toolpath(
    toolpath: Iterable[Sequence[float]],
    profile: ArmKinematicProfile,
    controller_config: ControllerConfig | None = None,
    *,
    robot_mode: bool = True,
    mode: str = "xyzabc",
    postprocessor: PostProcessor | None = None,
) -> List[str]:
    """Export a toolpath for robotic arm or standard CNC.

    Parameters
    ----------
    toolpath : iterable of coordinate sequences
        Either XYZ or XYZABC points.
    profile : ArmKinematicProfile
        Robotic arm kinematic description.
    controller_config : ControllerConfig, optional
        Required when ``robot_mode`` is ``False`` to produce CNC G-code.
    robot_mode : bool, optional
        When ``False`` fallback to :func:`toolpath_to_gcode` for CNC output.
    mode : str, optional
        ``"xyzabc"`` for extended G-code or ``"joint"`` for MOVE_ARM commands.
    postprocessor : PostProcessor, optional
        Custom formatting of robotic commands.

    Returns
    -------
    list of str
        Generated command lines.
    """

    pp = postprocessor or PostProcessor()

    if not robot_mode:
        if controller_config is None:
            raise ValueError("controller_config required when robot_mode=False")
        logger.info("Exporting CNC G-code, %d points", len(list(toolpath)))
        return toolpath_to_gcode(list(toolpath), controller_config)

    lines: List[str] = []
    for pt in toolpath:
        pose = tuple(pt)
        if len(pose) < 3:
            logger.warning("Point %s has insufficient dimensions", pose)
            continue
        joints = profile.workspace_to_joints(pose[:3])
        if mode == "xyzabc":
            # Pad pose to 6 axes
            pose6 = pose + (0.0,) * (6 - len(pose))
            lines.append(pp.format_pose(pose6))
        elif mode == "joint":
            lines.append(pp.format_joints(joints))
        else:
            raise ValueError("mode must be 'xyzabc' or 'joint'")
    logger.info("Exported %d robotic lines in mode %s", len(lines), mode)
    return lines


def stream_robotic_toolpath(
    toolpath: Iterable[Sequence[float]],
    profile: ArmKinematicProfile,
    port: str,
    baud: int = 115200,
    *,
    mode: str = "xyzabc",
) -> None:
    """Export and stream a toolpath directly to the robot controller.

    Parameters
    ----------
    toolpath : iterable of coordinate sequences
        XYZ or XYZABC points to send.
    profile : ArmKinematicProfile
        Kinematic description of the robot.
    port : str
        Serial device, e.g. ``/dev/ttyUSB0``.
    baud : int, optional
        Serial baudrate. Defaults to 115200.
    mode : str, optional
        ``"xyzabc"`` for extended G-code or ``"joint"`` for ``MOVE_ARM``.
    """

    lines = export_toolpath(toolpath, profile, robot_mode=True, mode=mode)
    logger.info("Streaming %d lines to %s", len(lines), port)
    tmp = tempfile.NamedTemporaryFile("w", delete=False, suffix=".gcode")
    try:
        tmp.write("\n".join(lines))
        tmp.close()
        stream_gcode_to_grbl(tmp.name, port, baud)
        logger.info("Robot run completed")
    except Exception as exc:
        logger.error("Robot run failed: %s", exc)
        raise
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass
