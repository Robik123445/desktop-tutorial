"""Legacy wrapper for AI optimization utilities."""

from .feedrate_optimizer import optimize_toolpath_based_on_surface
from .gcode_cleaner import (
    optimize_toolpath,
    optimize_toolpath_sequence,
    optimize_toolpath_sequence_tsp,
)
from .trajectory_planner import smooth_toolpath_corners

__all__ = [
    "optimize_toolpath_based_on_surface",
    "optimize_toolpath",
    "optimize_toolpath_sequence",
    "optimize_toolpath_sequence_tsp",
    "smooth_toolpath_corners",
]
