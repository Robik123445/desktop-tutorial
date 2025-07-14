diff --git a/cam_slicer/__init__.py b/cam_slicer/__init__.py
index 80a42f2dec06041f3e22e618aaec563b66c467bd..118a73205214097fed867e5ca55f79d44652b596 100644
--- a/cam_slicer/__init__.py
+++ b/cam_slicer/__init__.py
@@ -89,50 +89,55 @@ from .digital_twin import DigitalTwin, WorkshopTwin
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
diff --git a/cam_slicer/__init__.py b/cam_slicer/__init__.py
index 80a42f2dec06041f3e22e618aaec563b66c467bd..118a73205214097fed867e5ca55f79d44652b596 100644
--- a/cam_slicer/__init__.py
+++ b/cam_slicer/__init__.py
@@ -334,26 +339,29 @@ __all__ = [
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
 ]
