"""Robotic arm helpers."""

from .interface import ArmKinematicProfile, format_extended_g1, format_move_arm, export_robotic_toolpath
from .pipeline import convert_xyz_toolpath, batch_convert, stream_converted_toolpath
from .trajectory_optimizer import optimize_robotic_trajectory
from .base_optimizer import (
    optimize_base_position,
    optimize_base_position_ai,
    plan_base_zones,
    plan_workspace_zones,
    BaseZone,
    ZonePlan,
)
from .kinematics import (
    TransformConfig,
    transform_point,
    apply_pressure_advance,
    apply_input_shaping,
    calculate_junction_velocity,
    plan_feedrate_with_lookahead,
    axis_limits,
)
from .safety import emergency_stop
from .sensor_stream import SensorStream
from .live_feedback_engine import LiveFeedbackEngine
from .trajectory_corrector import TrajectoryCorrector
from .joint_load_predictor import JointLoadPredictor
from .collision_alerts import CollisionAlerts
from .singularity_map import map_singularity_zones
from .head_selector import HeadSelector
from .operation_mapper import map_operations
from .workstation_monitor import start_live_diagnostics

__all__ = [
    "ArmKinematicProfile",
    "format_extended_g1",
    "format_move_arm",
    "export_robotic_toolpath",
    "convert_xyz_toolpath",
    "batch_convert",
    "stream_converted_toolpath",
    "optimize_robotic_trajectory",
    "optimize_base_position",
    "optimize_base_position_ai",
    "plan_base_zones",
    "plan_workspace_zones",
    "BaseZone",
    "ZonePlan",
    "TransformConfig",
    "transform_point",
    "apply_pressure_advance",
    "apply_input_shaping",
    "calculate_junction_velocity",
    "plan_feedrate_with_lookahead",
    "axis_limits",
    "emergency_stop",
    "SensorStream",
    "LiveFeedbackEngine",
    "TrajectoryCorrector",
    "JointLoadPredictor",
    "CollisionAlerts",
    "HeadSelector",
    "map_operations",
    "map_singularity_zones",
    "start_live_diagnostics",
]
