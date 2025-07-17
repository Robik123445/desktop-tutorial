import logging
from typing import Iterable, List, Sequence, Callable

from cam_slicer.logging_config import setup_logging
from .interface import ArmKinematicProfile, format_extended_g1, format_move_arm

setup_logging()
logger = logging.getLogger(__name__)

def convert_xyz_toolpath(
    toolpath: Iterable[Sequence[float]],
    profile: ArmKinematicProfile,
    *,
    obstacles=None,
    mode: str = "xyzabc",
) -> List[str]:
    logger.info("Converting toolpath with %d points using profile %s", len(list(toolpath)), profile.name)
    toolpath = list(toolpath)
    lines: List[str] = []
    for pt in toolpath:
        pose = pt if len(pt) >= 6 or profile.dh_params else pt[:3]
        try:
            joints = profile.workspace_to_joints(pose)
        except Exception as exc:
            logger.warning("Unreachable pose %s: %s", pose, exc)
            continue
        if profile.check_collision(joints, obstacles):
            logger.warning("Collision detected at pose %s", pose)
            continue
        if len(joints) >= 2 and abs(abs(joints[1]) - 180) < 1e-2:
            logger.warning("Possible singularity near pose %s", pose)
        if mode == "xyzabc":
            p6 = tuple(pose) + (0.0,) * (6 - len(pose))
            lines.append(format_extended_g1(*p6))
        elif mode == "joint":
            lines.append(format_move_arm(joints))
        else:
            raise ValueError("mode must be 'xyzabc' or 'joint'")
    logger.info("Converted %d lines", len(lines))
    return lines

def batch_convert(
    toolpaths: Iterable[Iterable[Sequence[float]]],
    profile: ArmKinematicProfile,
    *,
    obstacles=None,
    mode: str = "xyzabc",
) -> List[List[str]]:
    return [convert_xyz_toolpath(tp, profile, obstacles=obstacles, mode=mode) for tp in toolpaths]

def stream_converted_toolpath(
    toolpath: Iterable[Sequence[float]],
    profile: ArmKinematicProfile,
    sender: Callable[[str], None],
    *,
    obstacles=None,
    mode: str = "xyzabc",
) -> None:
    for line in convert_xyz_toolpath(toolpath, profile, obstacles=obstacles, mode=mode):
        sender(line)
