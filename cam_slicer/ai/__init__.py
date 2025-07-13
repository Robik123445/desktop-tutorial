"""AI utilities for CAM Slicer."""

from .toolpath_predictor import ToolpathPredictor
from .material_simulator import MaterialSimulator
from .roughness_predictor import predict_surface_roughness
from .feedback_advisor import AIFeedbackAdvisor
from .fillet_detection import detect_fillet_features, generate_fillet_tracing
from .rest_machining import analyze_remaining_area, generate_rest_machining_paths
from .feature_machining import detect_mesh_features, generate_feature_toolpaths
from .adaptive_steps import compute_adaptive_steps
from .chip_coolant import ChipCoolantAdvisor, apply_chip_evacuations
from .surface_texturing import generate_surface_texture

__all__ = [
    "ToolpathPredictor",
    "MaterialSimulator",
    "predict_surface_roughness",
    "AIFeedbackAdvisor",
    "detect_fillet_features",
    "generate_fillet_tracing",
    "analyze_remaining_area",
    "generate_rest_machining_paths",
    "detect_mesh_features",
    "generate_feature_toolpaths",
    "compute_adaptive_steps",
    "generate_surface_texture",
    "ForcePredictor",
    "simulate_force_profile",
    "ChipCoolantAdvisor",
    "apply_chip_evacuations",
]

from .force_prediction import ForcePredictor, simulate_force_profile
from .chip_coolant import ChipCoolantAdvisor, apply_chip_evacuations
