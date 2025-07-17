@@ -25,50 +25,51 @@
 from .utils import (
     blend_laser_mill,
     schedule_multi_head_job,
     prepare_hybrid_job,
     clip_toolpath_to_regions,
     Strategy,
     load_strategy,
     run_strategy,
     apply_heightmap_to_gcode,
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
 
@@ -347,26 +348,27 @@
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
     "feedrate_advisor",
     "trajectory_cleaner",
     "surface_comparator",
+    "validate_toolpath",
]
