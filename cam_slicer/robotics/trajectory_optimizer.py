import logging
from typing import List, Sequence, Tuple

from cam_slicer.logging_config import setup_logging
from .interface import ArmKinematicProfile
from cam_slicer.ai.gcode_cleaner import optimize_toolpath

setup_logging()
logger = logging.getLogger(__name__)


def optimize_robotic_trajectory(
    toolpath: List[Sequence[float]],
    profile: ArmKinematicProfile,
    *,
    angle_margin: float = 5.0,
    singularity_margin: float = 10.0,
) -> Tuple[List[Sequence[float]], List[str]]:
    """Optimize robotic arm trajectory and report risky moves.

    Parameters
    ----------
    toolpath : list of coordinate sequences
        Points defined in workspace coordinates.
    profile : ArmKinematicProfile
        Robot description used for inverse kinematics and limit checks.
    angle_margin : float, optional
        Degrees from joint limits considered risky.
    singularity_margin : float, optional
        Threshold from straight configuration to warn about singularity.

    Returns
    -------
    optimized_toolpath : list of sequence
        Smoothed toolpath with potential air moves removed.
    warnings : list of str
        Human readable warnings for problematic sections.
    """

    if not toolpath:
        return [], []

    # Basic smoothing using existing optimizer
    segments = optimize_toolpath([tuple(p[:3]) for p in toolpath])
    optimized: List[Sequence[float]] = [pt for seg in segments for pt in seg]

    warnings: List[str] = []
    joint_history: List[List[float]] = []

    for idx, pose in enumerate(optimized):
        try:
            joints = profile.workspace_to_joints(pose[:3])
            joint_history.append(joints)
        except Exception as exc:  # unreachable pose
            logger.warning("Unreachable pose %s: %s", pose, exc)
            warnings.append(f"Pose {idx} unreachable: {exc}")
            continue

        for j_idx, angle in enumerate(joints):
            lo, hi = profile.joint_limits[j_idx]
            if angle <= lo + angle_margin or angle >= hi - angle_margin:
                message = f"Joint {j_idx + 1} near limit at point {idx}"
                logger.warning(message)
                warnings.append(message)
        if len(joints) > 1 and abs(abs(joints[1]) - 180) < singularity_margin:
            msg = f"Possible singularity near point {idx}"
            logger.warning(msg)
            warnings.append(msg)
        if idx > 0:
            prev = joint_history[idx - 1]
            delta = max(abs(a - b) for a, b in zip(prev, joints))
            if delta > 90:
                msg = f"Large joint change at point {idx}"
                logger.warning(msg)
                warnings.append(msg)

    logger.info("Optimized %d points with %d warnings", len(optimized), len(warnings))
    return optimized, warnings
