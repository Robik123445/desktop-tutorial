diff --git a/cam_slicer/__init__.py b/cam_slicer/__init__.py
index 03ee3c9535b6f0ea8eb4e71798ef587568c16dc4..befd03be642ab551cf8f5d433874d02e8bb74fda 100644
--- a/cam_slicer/__init__.py
+++ b/cam_slicer/__init__.py
@@ -170,50 +170,51 @@ from .robotics.pipeline import (
     stream_converted_toolpath,
 )
 from .robotics.diagnostics import log_run, log_warning, log_error, log_feedback
 from .robotics.trajectory_optimizer import optimize_robotic_trajectory
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
 from .toolpath_simulator import parse_toolpath, plot_toolpath, simulate_toolpath
+from .heightmap import HeightMap, apply_heightmap_to_gcode
 
 __all__ = [
     "_get_header_footer",
     "process_toolpath",
     "toolpath_to_gcode",
     "ControllerConfig",
     "nest_shapes",
     "Placement",
     "ZMap",
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
diff --git a/cam_slicer/__init__.py b/cam_slicer/__init__.py
index 03ee3c9535b6f0ea8eb4e71798ef587568c16dc4..befd03be642ab551cf8f5d433874d02e8bb74fda 100644
--- a/cam_slicer/__init__.py
+++ b/cam_slicer/__init__.py
@@ -346,26 +347,28 @@ __all__ = [
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
     "feedrate_advisor",
     "trajectory_cleaner",
     "surface_comparator",
     "parse_toolpath",
     "plot_toolpath",
     "simulate_toolpath",
+    "HeightMap",
+    "apply_heightmap_to_gcode",
 ]
