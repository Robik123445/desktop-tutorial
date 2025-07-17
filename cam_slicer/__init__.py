diff --git a/cam_slicer/__init__.py b/cam_slicer/__init__.py
index 80a42f2dec06041f3e22e618aaec563b66c467bd..ad9f5162708cf6c79811372ee6c622165f5c5810 100644
--- a/cam_slicer/__init__.py
+++ b/cam_slicer/__init__.py
@@ -1,138 +1,149 @@
 """Convenience imports for CAM Slicer.
 
 The package configures central logging on import so all modules share the same
 `logs/central.log` file.  Importers need not call :func:`setup_logging` again.
 """
 
 from .logging_config import setup_logging, setup_night_logging
 from .performance import profiled, parallel_map
 
 setup_logging()
 
 from .core.header_footer import _get_header_footer, ControllerConfig
 from .core.gcode_export import toolpath_to_gcode
+from .core.model_io import import_model, export_gcode
 from .core.engine import process_toolpath
 from .utils import (
     nest_shapes,
     Placement,
     ZMap,
     GeoFence,
     get_active_geofence,
     reset_active_geofence,
     assign_hybrid_operations,
     blend_additive_subtractive,
     blend_laser_mill,
     schedule_multi_head_job,
     prepare_hybrid_job,
     clip_toolpath_to_regions,
     Strategy,
     load_strategy,
     run_strategy,
+    apply_heightmap_to_gcode,
 )
 from .utils.batch_processor import run_macro_sequence, batch_process_toolpaths
 from .shapes import generate_adaptive_path
 from .ai.adaptive_steps import compute_adaptive_steps
 from .ai.surface_texturing import generate_surface_texture
 from .shapes import generate_spiral_helix
 from .shapes import morph_between_curves
 from .visualizer import (
     preview_gcode,
     parse_gcode,
     render_surface_and_toolpath,
     align_toolpath_to_surface,
     simulate_motion_over_surface,
     analyze_toolpath_vs_surface,
     export_preview_image,
     backplot_gcode,
 )
+from .robotics.safety import validate_toolpath
 from .importers import (
     import_dwg,
     import_obj,
     import_stl,
     analyze_dxf,
     analyze_stl,
     mesh_simplify,
     import_and_vectorize_image,
     import_svg,
 )
 from .core.mesh_strategies import (
     generate_roughing_paths,
     generate_finishing_paths,
     generate_carving_paths,
     generate_zlevel_roughing_paths,
     generate_zlevel_finishing_paths,
     generate_parallel_finishing_paths,
     generate_offset_finishing_paths,
     generate_high_polish_paths,
     generate_radial_finishing_paths,
     project_curves_to_surface,
     plan_mesh_operations,
 )
 from .cli_wizard import cli
 
 try:  # optional dependency
     from .api_server import create_app
 except Exception:  # pragma: no cover - missing FastAPI
     create_app = None
 from .sensors import generate_heightmap, export_heightmap_to_json
+from .probing import probe_heightmap
 from .sender import (
     stream_gcode_live,
     stream_gcode_to_grbl,
     LiveGcodeStreamer,
     stream_gcode_with_feedback,
     stream_with_recovery,
     resume_job,
     save_checkpoint,
     load_checkpoint,
 )
 from .digital_twin import DigitalTwin, WorkshopTwin
 from .ros_transform_adapter import (
     transform_config_to_pose,
     toolpath_to_pose_list,
     gcode_to_pose_list,
     pose_to_transform_config,
     compose_transforms,
     invert_transform,
     map_point_between_frames,
     ROSTransformAdapter,
 )
 from .config.macros import get_macro
 from .config import get_machine_config
 from .ai.feedrate_optimizer import optimize_toolpath_based_on_surface
 from .ai.gcode_cleaner import (
     optimize_toolpath,
     optimize_toolpath_sequence,
     optimize_toolpath_sequence_tsp,
 )
 from .ai.trajectory_planner import smooth_toolpath_corners
 from .ai.hsm import filter_small_arcs, optimize_high_speed_toolpath
 from .ai.feedback_advisor import AIFeedbackAdvisor
 from .ai.toolpath_predictor import ToolpathPredictor
 from .ai.material_simulator import MaterialSimulator
 from .ai.force_prediction import ForcePredictor, simulate_force_profile
 from .ai.roughness_predictor import predict_surface_roughness
+from .ai.analyzers import (
+    feedrate_advisor,
+    trajectory_cleaner,
+    surface_comparator,
+    plugin_optimizer,
+    ml_speed_optimizer,
+)
 from .ai.fillet_detection import detect_fillet_features, generate_fillet_tracing
 from .ai.rest_machining import analyze_remaining_area, generate_rest_machining_paths
 from .ai.debris import (
     detect_debris,
     add_debris_zones_from_image,
     plan_toolpath_avoiding_debris,
 )
 from .ai.chip_coolant import ChipCoolantAdvisor, apply_chip_evacuations
 from .vision.debris_detector import detect_objects, run_live_detection
 from .vision.board_detector import (
     detect_board_position,
     get_transform_from_detection,
     auto_transform_gcode,
     process_camera_to_gcode,
 )
 from .plugin_manager import load_plugins, reload_plugins, get_plugin, get_all_plugins
 from .plugin_manager import execute_plugin
 from .plugin_marketplace import (
     list_available_plugins,
     install_plugin,
     update_plugin,
     remove_plugin,
 )
 from .post_processors import (
     load_post_processors,
@@ -169,93 +180,97 @@ from .robotics.trajectory_optimizer import optimize_robotic_trajectory
 from .robotics.base_optimizer import (
     optimize_base_position,
     optimize_base_position_ai,
     plan_base_zones,
     plan_workspace_zones,
     BaseZone,
     ZonePlan,
 )
 from .robotics.kinematics import (
     TransformConfig,
     transform_point,
     apply_pressure_advance,
     apply_input_shaping,
     calculate_junction_velocity,
     plan_feedrate_with_lookahead,
     axis_limits,
 )
 from .robotics.safety import emergency_stop
 from .machines import Machine, MachineManager, Job
 from .user_manager import UserManager, User
 
 __all__ = [
     "_get_header_footer",
     "process_toolpath",
     "toolpath_to_gcode",
+    "import_model",
+    "export_gcode",
     "ControllerConfig",
     "nest_shapes",
     "Placement",
     "ZMap",
+    "apply_heightmap_to_gcode",
     "preview_gcode",
     "parse_gcode",
     "render_surface_and_toolpath",
     "align_toolpath_to_surface",
     "simulate_motion_over_surface",
     "analyze_toolpath_vs_surface",
     "export_preview_image",
     "backplot_gcode",
     "TransformConfig",
     "transform_point",
     "apply_pressure_advance",
     "apply_input_shaping",
     "calculate_junction_velocity",
     "plan_feedrate_with_lookahead",
     "axis_limits",
     "GeoFence",
     "get_active_geofence",
     "reset_active_geofence",
     "assign_hybrid_operations",
     "blend_additive_subtractive",
     "blend_laser_mill",
     "schedule_multi_head_job",
     "prepare_hybrid_job",
     "clip_toolpath_to_regions",
     "Strategy",
     "load_strategy",
     "run_strategy",
     "run_macro_sequence",
     "batch_process_toolpaths",
     "generate_adaptive_path",
     "compute_adaptive_steps",
     "generate_spiral_helix",
     "morph_between_curves",
     "get_macro",
     "get_machine_config",
     "ToolDatabase",
     "ToolMaterialDB",
     "generate_heightmap",
     "export_heightmap_to_json",
+    "probe_heightmap",
     "stream_gcode_live",
     "stream_gcode_to_grbl",
     "LiveGcodeStreamer",
     "stream_gcode_with_feedback",
     "stream_with_recovery",
     "resume_job",
     "save_checkpoint",
     "load_checkpoint",
     "optimize_toolpath_based_on_surface",
     "optimize_toolpath",
     "optimize_toolpath_sequence",
     "optimize_toolpath_sequence_tsp",
     "smooth_toolpath_corners",
     "filter_small_arcs",
     "optimize_high_speed_toolpath",
     "detect_debris",
     "add_debris_zones_from_image",
     "plan_toolpath_avoiding_debris",
     "AIFeedbackAdvisor",
     "ToolpathPredictor",
     "MaterialSimulator",
     "ForcePredictor",
     "simulate_force_profile",
     "ChipCoolantAdvisor",
     "apply_chip_evacuations",
@@ -334,26 +349,32 @@ __all__ = [
     "mesh_simplify",
     "generate_roughing_paths",
     "generate_finishing_paths",
     "generate_carving_paths",
     "generate_zlevel_roughing_paths",
     "generate_zlevel_finishing_paths",
     "generate_parallel_finishing_paths",
     "generate_offset_finishing_paths",
     "generate_high_polish_paths",
     "generate_radial_finishing_paths",
     "project_curves_to_surface",
     "plan_mesh_operations",
     "import_and_vectorize_image",
     "import_svg",
     "cli",
     "create_app",
     "Machine",
     "MachineManager",
     "Job",
     "UserManager",
     "User",
     "setup_logging",
     "setup_night_logging",
     "profiled",
     "parallel_map",
+    "feedrate_advisor",
+    "trajectory_cleaner",
+    "surface_comparator",
+    "plugin_optimizer",
+    "ml_speed_optimizer",
+    "validate_toolpath",
 ]
