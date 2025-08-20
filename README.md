# CAM Slicer
[![CI](https://github.com/example/cam_slicer/actions/workflows/tests.yml/badge.svg)](https://github.com/example/cam_slicer/actions/workflows/tests.yml) [![Docs](https://github.com/example/cam_slicer/actions/workflows/docs.yml/badge.svg)](https://example.github.io/cam_slicer/)
Full documentation: https://example.github.io/cam_slicer/



Simple engine to generate headers and footers for CNC controllers and to modify toolpaths.

## Usage

Create `ControllerConfig` with `CONTROLLER_TYPE` set to supported type (`grbl` or `smoothie`) and call `_get_header_footer()`.
Toolpaths can be processed with `process_toolpath()` using different cutting strategies.

```
from cam_slicer.core.header_footer import _get_header_footer, ControllerConfig
from cam_slicer.core.engine import process_toolpath

cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
header, footer = _get_header_footer(cfg)

toolpath = [(0, 0, -1), (10, 0, -1)]
processed = process_toolpath(toolpath, cutting_strategy="climb")
```

Logs are stored in `logs/central.log`.
The file rotates automatically when it exceeds 1&nbsp;MB. Use
`export_critical_logs()` to email or send all warnings and errors to Slack if
required.
If you want nightly logs, call `setup_night_logging()` which writes to `logs/night.log` and rotates at midnight.

```python
from cam_slicer.logging_config import export_critical_logs

report = export_critical_logs(email="ops@example.com")
```

## Digital twin

`cam_slicer.digital_twin` provides lightweight `DigitalTwin` and `WorkshopTwin`
classes for deterministic simulation and monitoring. Processed toolpath points
are stored in the module-level `preview_points` list and all operations are
logged to `log.txt` for easy diagnostics.

`cam_slicer.core.header_footer` provides controller-specific headers and
footers, while `cam_slicer.core.gcode_export` handles G-code formatting.

`toolpath_to_gcode` converts a list of points to gcode. A `TransformConfig` rotates,
scales and offsets each point. It also honours work/tool offsets (`G54`-`G59`)
and coordinate shifts. `transform_point` supports `mode='polar'` for ``(r, theta)`` inputs.

```
from cam_slicer.core.gcode_export import toolpath_to_gcode
from cam_slicer.motion.kinematics import TransformConfig

cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
transform = TransformConfig(rotation_deg=90, scale=1.0, offset=(0, 10, 0))
gcode = toolpath_to_gcode(processed, cfg, transform, feedrate=1200)
```

Set ``feedrate`` to enable pressure advance compensation. The adjustment uses
a fixed acceleration of 500 mm/sÂ² and can be tuned with ``advance_factor``:

```python
gcode = toolpath_to_gcode(processed, cfg, transform,
                          feedrate=1200, advance_factor=0.02)
```

Enable experimental adaptive mode to modify feedrate based on a supplied
heightmap:

```python
gcode = toolpath_to_gcode(
    processed,
    cfg,
    transform,
    feedrate=1200,
    z_map=zmap,
    adaptive_mode=True,
)
```
This mode may produce unexpected results and is considered *experimental*.

Enable lookahead to smooth speed transitions. The queue size determines
how many upcoming points are inspected:

```python
gcode = toolpath_to_gcode(processed, cfg, transform,
                          feedrate=1200, lookahead=2)
```

Pass ``move_command='G0'`` to emit rapid moves after applying the same
transformations:

```
rapid = toolpath_to_gcode(processed, cfg, transform, move_command='G0')
```

Work offset and coordinate shift can be specified as well:

```
shifted = TransformConfig(work_offset=(5, 0, 0), shift=(0, 0, -1))
gcode = toolpath_to_gcode(processed, cfg, shifted)
```

Polar coordinates can be transformed by passing ``mode='polar'``:

```
from cam_slicer.motion.kinematics import transform_point

x, y, z = transform_point(10, 45, -1, transform, mode='polar')
```

### Arc moves

`toolpath_to_gcode` can emit circular arcs when ``shape='circle'`` and
``arc_support=True``. Provide points as ``[start, center, end]``. Orientation is
determined automatically.

```
circle_path = [(1, 0, -1), (0, 0, -1), (-1, 0, -1)]
gcode = toolpath_to_gcode(circle_path, cfg, transform, shape='circle', arc_support=True)
```

### Pressure advance

`apply_pressure_advance` adjusts feedrate based on acceleration to compensate
for filament pressure. When ``feedrate`` is given to ``toolpath_to_gcode`` the
function automatically applies this compensation using the configured
``advance_factor``.

```
from cam_slicer.motion.kinematics import apply_pressure_advance

compensated = apply_pressure_advance(1200, 100, 0.03)
```

### Junction velocity

`calculate_junction_velocity` returns a lowered feedrate at sharp corners based
on a junction deviation model.

```
from cam_slicer.motion.kinematics import calculate_junction_velocity

speed = calculate_junction_velocity((1, 0, 0), (0, 1, 0), 1000, 0.05)
```

### Input shaping

`apply_input_shaping` filters motion points to minimise oscillations.

```
from cam_slicer.motion.kinematics import apply_input_shaping

points = [(0,0,0), (1,0,0), (2,0,0)]
shaped = apply_input_shaping(points, frequency=20, damping=0.05)
```

### Lookahead planning

`plan_feedrate_with_lookahead` uses a queue of upcoming points to limit the
feedrate before entering sharp corners.

```
from cam_slicer.motion.kinematics import plan_feedrate_with_lookahead

path = [(0,0,0), (1,0,0), (1,1,0)]
speeds = plan_feedrate_with_lookahead(path, 1000, 1000, 0.05, queue_size=2)
```

### Axis limits

`transform_point` can clamp motion using per-axis limits. Pass the previous
position and velocity so motion adheres to the ``axis_limits`` dictionary.

```python
from cam_slicer.motion.kinematics import transform_point, axis_limits, TransformConfig

prev = (0, 0, 0)
prev_v = (0, 0, 0)
cfg = TransformConfig()

pos, vel = transform_point(1000, 0, 0, cfg,
                           prev_point=prev,
                           prev_velocity=prev_v,
                           dt=1.0,
                           axis_cfg=axis_limits,
                           return_velocity=True)
print(pos)
```

### Machine presets

``get_machine_config`` returns ready-made settings for common machines. The
dictionary includes axis limits, controller type and dynamic parameters.

```python
from cam_slicer.config.machine_config import get_machine_config

machine = get_machine_config("MACHINE_GRBL_SMALL")
print(machine["axis_limits"])
```

### Z-map compensation

`ZMap` can load a CSV or JSON grid with probed surface heights. Pass an
instance to `toolpath_to_gcode` so every Z move is offset by the map.

```python
from cam_slicer.utils import ZMap

zmap = ZMap(points=[(0, 0, 0.1), (100, 0, 0.2)])
gcode = toolpath_to_gcode(toolpath, cfg, transform, z_map=zmap)
```




### Nesting shapes

`nest_shapes` arranges closed polygons onto a sheet using a greedy first-fit
strategy. Shapes are sorted by area and placed left-to-right in rows. Rotation
by 90Â° is attempted when it helps the fit. The function returns placement
coordinates so you can generate G-code or previews.

```python
from cam_slicer.utils import nest_shapes

shapes = [
    [(0, 0), (2, 0), (2, 1), (0, 1)],
    [(0, 0), (1, 0), (1, 2), (0, 2)],
]
placements = nest_shapes(shapes, sheet_width=3, sheet_height=3)
for p in placements:
    print(p)
```


### G-code preview

Use `preview_gcode` to plot toolpaths. When `laser_mode=True` a 2D plot is generated,
otherwise a 3D plot showing Z depth is returned.

```python
from cam_slicer.visualizer import preview_gcode

gcode = ["G1 X0 Y0 Z0", "G1 X1 Y0 Z-1"]
fig = preview_gcode(gcode, laser_mode=False)
```

Call `export_preview_image` to save the preview with a scale bar and part ID:

```python
from cam_slicer.visualizer import export_preview_image

export_preview_image("path.gcode", "preview.png")
```

### G-code backplot & replay

```python
from cam_slicer.visualizer import backplot_gcode

ani = backplot_gcode("path.gcode")
```
Press space during playback to pause. Use the right arrow to step.


### Surface overlay

`render_surface_and_toolpath` combines a scanned surface with a toolpath in a 3D view.

```python
from cam_slicer.visualizer import render_surface_and_toolpath

fig = render_surface_and_toolpath("map.json", "path.gcode")
```

### Motion simulation

`simulate_motion_over_surface` animates how the toolhead moves over a scanned
surface. Use the space bar to pause/resume and `n` to step through frames.

```python
from cam_slicer.visualizer import simulate_motion_over_surface

ani = simulate_motion_over_surface("map.json", "path.gcode", step_time=0.05)
```

### Toolpath analysis

`analyze_toolpath_vs_surface` returns a list of collision or noâcontact points
when you compare a heightmap to the actual Gâcode path.

```python
from cam_slicer.visualizer import analyze_toolpath_vs_surface

hmap = {"points": [
    {"x": 0, "y": 0, "z": 0},
    {"x": 1, "y": 0, "z": 0},
]}
issues = analyze_toolpath_vs_surface(hmap, "path.gcode")
print(issues)
```

### Toolpath optimization

`optimize_toolpath_based_on_surface` adjusts feedrates using a scanned heightmap.

```python
from cam_slicer.ai.feedrate_optimizer import optimize_toolpath_based_on_surface

optimized_path = optimize_toolpath_based_on_surface("path.gcode", "map.json")
print("Saved to", optimized_path)
```

Use `optimize_toolpath` to smooth corners and remove short air moves before
generating G-code:

```python
from cam_slicer.ai.gcode_cleaner import optimize_toolpath

path = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (2, 1, 0.5)]
segments = optimize_toolpath(path)
```

`smooth_toolpath_corners` inserts explicit arc commands when a corner exceeds
the chosen angle. The output list mixes plain points with arc dictionaries so
the motion planner can generate G2/G3 moves directly.

```python
from cam_slicer.ai.trajectory_planner import smooth_toolpath_corners

smoothed = smooth_toolpath_corners(path, angle_threshold=45)
```

`optimize_toolpath_sequence` reorders multiple toolpath segments so the cutter
travels the shortest distance between cuts. The function inserts rapid
connections between segments in the optimal order.

```python
from cam_slicer.ai.gcode_cleaner import optimize_toolpath_sequence

paths = [
    [(0, 0, 0), (1, 0, 0)],
    [(5, 0, 0), (5, 1, 0)],
    [(2, 0, 0), (2, 1, 0)],
]
ordered = optimize_toolpath_sequence(paths)
```
For complex jobs you can try a slower 2-opt TSP solver:

```python
from cam_slicer.ai.gcode_cleaner import optimize_toolpath_sequence_tsp

ordered = optimize_toolpath_sequence_tsp(paths)
```

### High-speed machining

`optimize_high_speed_toolpath` combines corner smoothing, lookahead planning
and arc filtering to generate smooth G-code ideal for HSM machines.

```python
from cam_slicer.ai.hsm import optimize_high_speed_toolpath
from cam_slicer.core.gcode_export import ControllerConfig

raw = [(0, 0, 0), (10, 0, 0), (10, 5, 0), (20, 5, 0)]
cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
gcode = optimize_high_speed_toolpath(raw, cfg, feedrate=2000)
```

### Toolpath alignment

`align_toolpath_to_surface` fits a toolpath from an STL or SVG file onto a scanned surface using a basic ICP routine.

```python
from cam_slicer.visualizer import align_toolpath_to_surface

aligned, surface = align_toolpath_to_surface("tool.stl", "map.json")
```

### Adaptive path generation

`generate_adaptive_path` creates a spiral or zigzag toolpath. ``stepover`` and
``engagement_angle`` combine to keep a nearly constant radial engagement while
``stepdown`` controls each Z layer. Pass ``adaptive_mode=True`` to enable
experimental surface following where the stepdown is reduced on deeper layers.
Use with caution as results may vary.

```python
from cam_slicer import generate_adaptive_path

boundary = [(10, 0), (0, 10), (-10, 0), (0, -10)]
segments = generate_adaptive_path(boundary, depth=-2, stepdown=1, stepover=0.5)
```

### Adaptive stepdown & stepover

``compute_adaptive_steps`` returns tuned parameters based on material hardness,
geometry complexity and current tool load.

```python
from cam_slicer.ai import compute_adaptive_steps

stepdown, stepover = compute_adaptive_steps(
    1.0, 0.5, material_hardness=2.0, geometry_factor=0.6, tool_load=0.8
)
segments = generate_adaptive_path(
    boundary,
    depth=-3,
    stepdown=stepdown,
    stepover=stepover,
    adaptive_mode=True,
)
```

### Spiral helical milling

`generate_spiral_helix` produces a continuous helical ramp useful for pocketing or boring without retractions.

```python
from cam_slicer import generate_spiral_helix

spiral = generate_spiral_helix((0, 0), radius=5, depth=-3, pitch=1)
```

### Macros

User-defined macros can be inserted into the generated G-code. The macros are
stored in `config/macros.json` and referenced by name when calling
`toolpath_to_gcode`.

```python
from cam_slicer import toolpath_to_gcode, ControllerConfig, TransformConfig

cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
toolpath = [(0, 0, 0), (1, 0, 0)]
gcode = toolpath_to_gcode(
    toolpath,
    cfg,
    TransformConfig(),
    start_macro="START_SPINDLE",
    mid_macro="PAUSE",
    end_macro="END_SPINDLE",
)
```

### Automation macros & batch processing

Use `run_macro_sequence` to fetch multiple macros at once and
`batch_process_toolpaths` to generate one G-code file from several toolpaths.

```python
from cam_slicer import batch_process_toolpaths, run_macro_sequence, ControllerConfig

paths = [[(0,0,-1)], [(1,0,-1)]]
cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
gcode = batch_process_toolpaths(
    paths,
    cfg,
    start_macro="START_SPINDLE",
    between_macro="PAUSE",
    end_macro="END_SPINDLE",
)
```

### Heightmap generation

`generate_heightmap` probes a grid using a provided function and returns a
dictionary with heights for each XY point. Use `export_heightmap_to_json` to
save it for later processing.

```python
from cam_slicer.sensors import generate_heightmap, export_heightmap_to_json

def mock_probe(x, y):
    return x * 0.1 + y * 0.2

heightmap = generate_heightmap((0, 1), (0, 1), step=None, probe_func=mock_probe, density=3)
export_heightmap_to_json(heightmap, "map.json")
```

### DWG, STL and OBJ import

`import_dwg` loads 2D lines from DWG or DXF files and applies scaling or
rotation. `import_stl` and `import_obj` read mesh vertices from STL or OBJ
models. After loading a mesh you can run `analyze_stl` for a quick suitability
report and call `mesh_simplify` to reduce polygon count if needed.

```python
from cam_slicer.importers import (
    import_dwg,
    import_stl,
    analyze_stl,
    mesh_simplify,
)

profile = import_dwg("shape.dwg", scale=0.1, rotation_deg=90)
mesh = import_stl("model.stl")
report = analyze_stl("model.stl")
optimized = mesh_simplify("model.stl", target_polycount=10000)
repair = repair_mesh("model.stl", out_path="model_fixed.stl")
param = import_mesh_parametric(
    "model_fixed.stl", operations=[{"scale": 1.2}]
)
```

`repair_mesh` fixes holes, flipped normals and non-manifold edges. Use
`import_mesh_parametric` to apply basic operations like scaling or boolean
union before machining.

### 3D Machining Strategies

Use mesh-based roughing, finishing and carving paths for complex parts. Each
operation can use a different tool and the resulting toolpaths are ready for
preview in the viewport.

```python
from cam_slicer import (
    import_stl,
    generate_roughing_paths,
    generate_finishing_paths,
    generate_carving_paths,
    plan_mesh_operations,
)

mesh = import_stl("model.stl")
rough = generate_roughing_paths(mesh, stepdown=2.0, stepover=3.0)
finish = generate_finishing_paths(mesh)
ops = plan_mesh_operations(mesh, {"roughing": "8mm", "finishing": "3mm"})
```

For constant-Z passes you can use:

```python
from cam_slicer import (
    generate_zlevel_roughing_paths,
    generate_zlevel_finishing_paths,
)

rough_layers = generate_zlevel_roughing_paths(mesh, stepdown=1.0)
finish_layers = generate_zlevel_finishing_paths(mesh, stepdown=0.3)
```

For planar raster finishing along X or Y use:

```python
from cam_slicer import generate_parallel_finishing_paths

paths = generate_parallel_finishing_paths(mesh, stepover=0.5, axis="x")
```

To follow the surface with a constant offset between passes:

```python
from cam_slicer import generate_offset_finishing_paths

finish_offset = generate_offset_finishing_paths(
    mesh, offset=0.1, stepover=0.5, passes=3
)
```

For rotational parts you can create radial passes:

```python
from cam_slicer import generate_radial_finishing_paths

radial_paths = generate_radial_finishing_paths(mesh, angle_step=15)
```

### High-polish finishing

`generate_high_polish_paths` creates ultra-fine passes with small stepover so
surfaces reach mirror quality.

```python
from cam_slicer import generate_high_polish_paths

polish = generate_high_polish_paths(mesh, stepover=0.05, passes=5)
```
### Projected toolpaths

`project_curves_to_surface` projects 2D or 3D curves onto a mesh so engraving
or texturing follows the real surface.

```python
from cam_slicer import import_stl, project_curves_to_surface

mesh = import_stl("model.stl")
curve = [[(0.5, 0.0, 0.0), (0.5, 1.0, 0.0)]]
projected = project_curves_to_surface(curve, mesh, offset=0.0)
```

### Curve morphing

`morph_between_curves` creates intermediate curves between multiple boundaries.

```python
from cam_slicer import morph_between_curves

outer = [(0, 0, 0), (1, 0, 0)]
inner = [(0, 1, 0), (1, 1, 0)]
paths = morph_between_curves([outer, inner], layers=2)
```

### Pencil tracing with AI fillet detection

`generate_fillet_tracing` searches for tight corners and adds small finishing
arcs around fillets or grooves based on the cutter radius.

```python
from cam_slicer.ai import generate_fillet_tracing

rough = [(0, 0, 0), (1, 0, 0), (1, 1, 0)]
finishing = generate_fillet_tracing(rough, cutter_radius=1.0)
```

### AI rest machining

`generate_rest_machining_paths` computes leftover material after roughing and
creates cleanup passes with a smaller tool only where needed.

```python
from cam_slicer.ai import generate_rest_machining_paths

boundary = [(0,0), (40,0), (40,20), (0,20)]
rough_paths = generate_adaptive_path(boundary, depth=-2, stepdown=1, stepover=5)
rest = generate_rest_machining_paths(
    boundary,
    rough_paths,
    roughing_radius=5.0,
    rest_radius=2.0,
    depth=-2,
)
```

### Surface texturing

`generate_surface_texture` creates decorative patterns on a mesh using stippling,
wave or grayscale maps.

```python
from cam_slicer.ai import generate_surface_texture
import trimesh

mesh = trimesh.creation.box(extents=(10, 10, 2))
paths = generate_surface_texture(mesh, pattern="wave", spacing=1.0, amplitude=0.3)
```



### Image vectorization

`import_and_vectorize_image` converts bitmap images into vector paths. Use
`mode="outline"` to trace outer contours or `mode="centerline"` to generate a
single centerline using skeletonization. Basic SVG files can be loaded with
`import_svg`.

```python
from cam_slicer.importers import import_and_vectorize_image, import_svg

vectors = import_and_vectorize_image("logo.png", mode="outline")
svg_paths = import_svg("drawing.svg")
```

### Live streaming

`stream_gcode_live` streams a G-code file to a serial controller and waits for
`ok` after each line.

```python
from cam_slicer.sender import stream_gcode_live

stream_gcode_live("path.gcode", "/dev/ttyUSB0", 115200)
```

`stream_gcode_to_grbl` is a convenience wrapper for GRBL controllers. It
defaults to 115200 baud and waits for ``ok`` after each command.

```python
from cam_slicer.sender import stream_gcode_to_grbl

stream_gcode_to_grbl("path.gcode", "/dev/ttyUSB0")
```

For interactive control with pause and resume, use `stream_gcode_interactive` or
run the module as a script:

```bash
python -m cam_slicer.sender.grbl_streamer path.gcode /dev/ttyUSB0 --baud 115200
# Press 'p' to pause/resume, 'q' to quit
```

If you prefer an object-oriented API, ``LiveGcodeStreamer`` exposes ``pause``,
``resume`` and ``stop`` methods:

```python
from cam_slicer.sender import LiveGcodeStreamer

streamer = LiveGcodeStreamer("path.gcode", "/dev/ttyUSB0")
streamer.stream()  # call pause/resume/stop from another thread or UI
```

### Machine Manager

Manage multiple CNC machines and run jobs in parallel or sequentially.

```python
from cam_slicer import Machine, MachineManager

mgr = MachineManager()
mgr.add_machine(Machine("A", "COM1"))
mgr.add_machine(Machine("B", "COM2"))
mgr.machines["A"].add_job("a.gcode")
mgr.machines["B"].add_job("b.gcode")
mgr.start_all_parallel()
# distribute many jobs optimally
mgr.assign_jobs_optimized(["a.gcode", "b.gcode", "c.gcode"], lambda p: 10.0)
```

#### Multi-machine and user management

Assign jobs based on machine capabilities and user permissions.

```python
from cam_slicer import Machine, MachineManager, Job, UserManager

mgr = MachineManager()
mgr.add_machine(Machine("router1", "COM1", machine_type="cnc", heads=["router", "laser"]))
mgr.add_machine(Machine("laser1", "COM2", machine_type="laser", heads=["laser"]))

jobs = [
    Job("cut.gcode", machine_type="laser", head="laser", priority=1),
    Job("mill.gcode", machine_type="cnc", head="router"),
]
mgr.assign_jobs(jobs)

users = UserManager()
users.add_user("alice", "operator")
if users.check_permission("alice", "run_job"):
    mgr.start_all_parallel()
```

Machine definitions live in ``cam_slicer/config/machines.json``.
Jobs can also be controlled from the CLI:

```bash
python -m cam_slicer.cli_machine_manager list
python -m cam_slicer.cli_machine_manager start-all demo.gcode
```

### Job recovery

`stream_with_recovery` writes progress to `last_block.json` so a job can resume
after a power failure. Use `resume_job` once the machine is safe to continue.

```python
from cam_slicer.sender import stream_with_recovery, resume_job

# initial run
stream_with_recovery("path.gcode", "/dev/ttyUSB0")

# later, after restart
resume_job("path.gcode", "/dev/ttyUSB0")
```

### System report

Call `generate_system_report` to scan the project and create a Markdown summary saved as `report.txt`. The report lists missing docstrings and tests and ends with a short summary.

```python
from cam_slicer.utils import generate_system_report

report_md = generate_system_report(".")
print(report_md)
```

### Digital Twin

`DigitalTwin` reads live status from the controller and logs deviations from the
planned toolpath.

```python
from cam_slicer import DigitalTwin, ROSBridge
import time

dummy = type("Dummy", (), {"readline": lambda self: "X0 Y0 Z0 F500 T1 A0"})()
twin = DigitalTwin(dummy)
twin.start_monitoring()
time.sleep(0.1)
state = twin.get_live_state()
twin.log_status()
twin_state_bridge = ROSBridge()
twin_state_bridge.attach_digital_twin(twin)
time.sleep(0.1)
twin.stop_monitoring()
```

### Workshop Twin

`WorkshopTwin` aggregates multiple `DigitalTwin` instances so the entire shop can be
monitored at once.

```python
from cam_slicer import WorkshopTwin

workshop = WorkshopTwin()
workshop.add_machine("router1", dummy)
workshop.add_machine("router2", dummy2)
workshop.start_all()
states = workshop.get_states()
workshop.stop_all()

# live diagnostics with load prediction
from cam_slicer.robotics import start_live_diagnostics
start_live_diagnostics(workshop)
```

### Live Feedback

Use ``SensorStream`` with ``LiveFeedbackEngine`` to automatically correct poses while running.

```python
from cam_slicer.robotics import SensorStream, LiveFeedbackEngine, ArmKinematicProfile

profile = ArmKinematicProfile(
    name="planar",
    link_lengths=[1, 1, 1],
    joint_types=["revolute", "revolute", "revolute"],
    joint_limits=[(-90, 90), (-90, 90), (-90, 90)],
)

engine = LiveFeedbackEngine(profile)
stream = SensorStream('/dev/ttyUSB0', callback=engine.process_pose)
stream.start()
```

``JointLoadPredictor`` estimates joint load from current angles and speeds.
Combine it with ``CollisionAlerts`` to stop the arm when entering a geofence.

### In-process measurement and correction

``stream_gcode_with_feedback`` streams G-code while probing or scanning after each move
and offsets subsequent Z commands based on the measured deviation.

```python
from cam_slicer.sender import stream_gcode_with_feedback

def probe(x, y):
    return 0.0  # replace with real probing or scanning code

stream_gcode_with_feedback("job.gcode", "/dev/ttyUSB0", probe)
```

### ROS2 Bridge

`ROSBridge` exposes bidirectional communication with ROS2. It publishes
position, state, job progress and now also heartbeat and warning messages while
subscribing to ``/robot/command`` for start/pause/abort instructions. All
messages are logged with timestamps for later audit. The bridge automatically
reconnects if ROS2 is restarted and can stream a ``DigitalTwin`` state on
``/robot/twin``.

Run the bridge in a background thread or via the CLI:

```python
from cam_slicer import ROSBridge

bridge = ROSBridge()
bridge.publish_position(0, 0, 0)
bridge.publish_state("idle")
bridge.publish_job_status("ready")
bridge.publish_feedback("tool loaded")
bridge.publish_heartbeat("alive")
bridge.publish_warning("overload")
```

Run diagnostics or send commands from the command line:

```bash
python -m cam_slicer.cli_ros_bridge send /robot/command start
```

### ROS2 Test Node

``ROSTestNode`` subscribes to all bridge topics and sends test commands to
verify connectivity. Run it before hooking up real hardware:

```bash
python -m cam_slicer.ros_test_node --simulate
```


### ROS Transform Adapter

`ROSTransformAdapter` converts internal transformations to ROS TF frames and
back.  It also maps points between frames and composes or inverts transforms.
Broadcast machine pose or an entire toolpath:

```python
from cam_slicer import ROSTransformAdapter, TransformConfig, transform_config_to_pose

cfg = TransformConfig(rotation_deg=90, offset=(1, 2, 0))
pose = transform_config_to_pose(cfg)
adapter = ROSTransformAdapter()
adapter.broadcast_transform(cfg, parent_frame="world", child_frame="machine")
poses = gcode_to_pose_list(["G1 X1 Y0 Z0", "G1 X1 Y1 Z0"], cfg)
adapter.broadcast_toolpath([(1,0,0),(1,1,0)], cfg, parent_frame="world")
```


### GeoFence

`GeoFence` keeps your toolpath away from clamps or hands. Combine manual zones and AI detections.

```python
from cam_slicer.utils import GeoFence

# 1. manual zones
forbidden_manual = [(10, 10, 20, 20)]
air_move_manual = [(0, 0, 5, 100)]
geo_fence = GeoFence(forbidden_manual, air_move_manual)

toolpath = [(0, 0, -1), (15, 15, -1), (65, 20, -1)]

# 2. AI detections from YOLO
ai_detections = [
    {"label": "clamp", "bbox": (60, 10, 70, 30)},
    {"label": "hand", "bbox": (90, 5, 110, 25)},
]
geo_fence.add_ai_zones_from_yolo(ai_detections, zone_type="forbidden")

# 3. filter toolpath and raise Z in air-move areas
filtered = geo_fence.filter_toolpath(toolpath)
adjusted = geo_fence.adjust_toolpath_for_air_moves(filtered)
```

### Debris mapping

`detect_debris` scans a camera image for leftover chips or tools. Use
`add_debris_zones_from_image` to update the `GeoFence` before running a job.

```python
from cam_slicer.ai.debris import (
    add_debris_zones_from_image,
    plan_toolpath_avoiding_debris,
)

remind = add_debris_zones_from_image("workspace.jpg", geo_fence)
safe_path = plan_toolpath_avoiding_debris(toolpath, geo_fence)
if remind:
    print("Clean table reminder")
```
Use the **Debris scan & avoid** button in the *Export & Streaming* panel of the UI to perform this check before running a job.

### Region-constrained machining

`clip_toolpath_to_regions` limits machining to user-selected polygons. Combine this with `GeoFence` for fine control.

```python
from cam_slicer.utils import clip_toolpath_to_regions

toolpath = [(0,0,-1),(10,0,-1),(10,10,-1),(0,10,-1)]
region = [(2,2),(8,2),(8,8),(2,8)]
clipped = clip_toolpath_to_regions(toolpath, [region])
```

### Feature-based machining

`detect_mesh_features` analyses a mesh for common features like pockets or holes and `generate_feature_toolpaths` creates matching paths automatically.

```python
from cam_slicer.ai import generate_feature_toolpaths
import trimesh

mesh = trimesh.creation.box(extents=(10, 10, 2))
paths = generate_feature_toolpaths(mesh, depth=-2, tool_diameter=2)
```


### Hybrid workspace scanning

`scan_workspace` combines a camera debris scan and a probing heightmap. Choose
`mode="camera"`, `"probe"` or `"hybrid"` and provide a probing function.

```python
from cam_slicer.workspace_scanner import scan_workspace, apply_scan_to_toolpath

def probe(x, y):
    return 0.0  # replace with real probing code

data = scan_workspace(mode="hybrid", probe_func=probe)
adjusted = apply_scan_to_toolpath(toolpath, data)
```

The viewport overlay shows red rectangles for debris zones and green points for
probing data so you can verify the combined map before machining.
`GeoFenceOverlay` visualizes these zones live in the UI and updates whenever a
new scan or model import occurs.

### Vision module

`detect_objects` uses the YOLOv8 nano model to find objects in an image. Provide a path to a picture or "0" to grab a single frame from the default camera. Bounding boxes are printed and logged to `logs/central.log`.

```python
from cam_slicer.vision.debris_detector import detect_objects

boxes = detect_objects("sample.jpg")
```

`detect_board_position` filters the YOLO detections to return only the board
bounding box.

```python
from cam_slicer.vision.board_detector import detect_board_position

info = detect_board_position("sample.jpg")
print(info)
```

`get_transform_from_detection` converts the detected board position into a
`TransformConfig` with the offset already in millimetres.

```python
from cam_slicer.vision.board_detector import get_transform_from_detection

transform = get_transform_from_detection(info)
```

`auto_transform_gcode` combines detection and transformation to produce ready-to-send G-code. When the detection
includes `width` and `height` the toolpath is scaled to fit the board.

```python
from cam_slicer.vision.board_detector import auto_transform_gcode

gcode_lines = auto_transform_gcode([(0, 0, -1), (10, 0, -1)], info)
for line in gcode_lines:
    print(line)
```

`process_camera_to_gcode` detects the board in an image, generates transformed
G-code and writes it straight to a file.

```python
from cam_slicer.vision.board_detector import process_camera_to_gcode

toolpath = [(0, 0, -1), (10, 0, -1)]
success = process_camera_to_gcode(toolpath, "sample.jpg", "out.gcode")
```

`run_live_detection` opens a webcam stream and shows detected objects live. Each
box is labeled with its class name and size converted to millimetres. If the
model detects a **person** or **hand**, a red box is drawn, a warning is printed
and `emergency_stop()` is called. Press `q` to exit.

```python
from cam_slicer.vision.debris_detector import run_live_detection

run_live_detection()
```

Detections for ``debris``, ``tool`` or ``clamp`` automatically update the shared
``GeoFence`` so later toolpaths can avoid those areas. When a *person* or *hand*
is detected the machine attempts an emergency stop.

### Plugins

Plugins extend CAM Slicer with custom processing steps. Every plugin is a Python
module inside `cam_slicer/plugins/` that implements `register()` returning a
`Plugin` dataclass.

Fields of ``Plugin``:

- ``name`` â unique identifier
- ``description`` â short summary
- ``apply`` â callable performing the work
- ``version`` â optional version string
- ``category`` â optional grouping

Example plugin:

```python
from cam_slicer.plugin_manager import Plugin

def register():
    def apply(path):
        return path[::-1]
    return Plugin(name="demo", description="reverse", apply=apply,
                  version="1.0", category="toolpath")
```

Use the plugin manager to load and query available plugins. These helpers are
available at package import time and fall back to no-ops if plugin dependencies
are missing, so importing ``cam_slicer`` never fails:

```python
from cam_slicer import load_plugins, reload_plugins, get_plugin, get_all_plugins

load_plugins()
for info in get_all_plugins():
    print(info["name"], "-", info["description"])

rev = get_plugin("reverse_path")
if rev:
    new_path = rev.apply([(0, 0, 0), (1, 0, 0)])

# Reload to pick up changes during development
reload_plugins()
```

Any import or registration errors are written to `logs/central.log` and invalid plugins
are skipped so that other plugins still load correctly.

Plugins execute in a sandboxed subprocess via `execute_plugin`. Previous versions are kept so failed updates can roll back safely.

```python
from cam_slicer import execute_plugin
execute_plugin("reverse_path", [[0, 0, 0], [1, 0, 0]])
```


## Plugin CLI
Use the CLI to manage plugins from the command line:
```bash
python cam_slicer/cli_plugin_manager.py list
python cam_slicer/cli_plugin_manager.py reload
python cam_slicer/cli_plugin_manager.py run reverse_path 1 2 3
# Additional arguments after the plugin name are forwarded to the plugin
```

### Batch Processing CLI

Combine multiple toolpaths with macros using the batch CLI:

```bash
python cam_slicer/cli_batch.py part1.gcode part2.gcode --start START_SPINDLE --between PAUSE --end END_SPINDLE
```

### Plugin Marketplace

Marketplace helpers allow installing or updating plugins distributed by the
community. Metadata files include ratings so you can choose the best option.

```python
from cam_slicer import list_available_plugins, install_plugin

for meta in list_available_plugins():
    print(meta["name"], meta.get("rating"))

install_plugin("example")
```

### Workflow Sharing

Complete CAM workflows can be exported to JSON and imported later or shared with
others.

```python
from cam_slicer import save_workflow, load_workflow

workflow = {"toolpaths": [[0,0,0], [1,0,0]], "material": "plywood"}
path = save_workflow(workflow, "demo")
restored = load_workflow(path)
```


## Web Dashboard
A React component for a simple CAM dashboard is located in `web/Dashboard.jsx`. It logs "New Project" clicks to `logs/central.log`.
The `FileUploader` component provides drag-and-drop file import and logs file selections along with the continue action.
See `web/README.md` for usage instructions.

The `ToolpathTransform` component provides rotation, scale and translation sliders.

```jsx
import ToolpathTransform from './web/ToolpathTransform';

<ToolpathTransform onChange={t => console.log(t)} />
```

Each change is logged to `logs/central.log`.



`GcodeStreamer` streams a file in small blocks and logs the progress. It displays the last streamed lines and provides start, pause, stop and restart controls.

```jsx
import GcodeStreamer from './web/GcodeStreamer';

<GcodeStreamer gcodeLines={['G1 X0 Y0', 'G1 X1 Y0']} fileName="demo.gcode" />
```

`WebcamDetector` displays a webcam preview after clicking **Start Object Detection** and triggers a red warning banner if a person or hand is seen. Actions are logged to `logs/central.log`.

```jsx
import WebcamDetector from './web/WebcamDetector';

<WebcamDetector />
```

`ProjectSaver` allows saving a project name with optional notes and exporting G-code. The action is logged and a short toast confirms the save.

```jsx
import ProjectSaver from './web/ProjectSaver';

<ProjectSaver gcodeLines={['G1 X0 Y0', 'G1 X1 Y0']} />
```


`ProjectManager` allows saving and loading full project data (toolpaths, material, machine profile, settings) as JSON. Actions are logged to `logs/central.log`.

```jsx
import ProjectManager from './web/ProjectManager';

<ProjectManager toolpaths={paths} material="MDF" machineProfile={profile} settings={{feed:1000}} />
```

`ResumeJobButton` reads the last checkpoint and lets you restart a job after
power loss.

```jsx
import ResumeJobButton from './web/ResumeJobButton';

<ResumeJobButton onResume={(info) => console.log('Resume', info)} />
```
`MaterialSelector` lets you choose a material and displays suggested spindle speed and feedrate. Selecting **Custom** opens an input for your own material. Each choice is logged to `logs/central.log`.

```jsx
import MaterialSelector from './web/MaterialSelector';

<MaterialSelector />
```

`MachineProfileSelector` lets you pick a predefined CNC machine preset or edit a **Custom** configuration. The chosen profile is saved to `localStorage` and logged.

```jsx
import MachineProfileSelector from './web/MachineProfileSelector';

<MachineProfileSelector />
```

`RobotArmPanel` lets you toggle robot arm mode, run dry runs for calibration and manage kinematic profiles with live warnings for overload or collisions. It can now stream the generated commands directly to your robot controller or export them for later use.
It also contains a diagnostics dashboard showing run statistics and a form to submit bug reports during beta testing.

```jsx
import RobotArmPanel from './web/RobotArmPanel';

<RobotArmPanel />
```

`CuttingStrategySelector` provides radio buttons for different milling approaches and logs the selection.

```jsx
import CuttingStrategySelector from './web/CuttingStrategySelector';

<CuttingStrategySelector />
```

`LayerPreview` shows sliced toolpath layers with a slider and play button.

```jsx
import LayerPreview from './web/LayerPreview';

const layers = [
  [[[0,0,0],[10,0,0]]],
  [[[0,0,0],[0,10,-1]]]
];

<LayerPreview layers={layers} />
```

`ToolList` lets you maintain a set of tools and saves them to the browser using `localStorage`. Actions are logged to `logs/central.log`.

```jsx
import ToolList from './web/ToolList';

<ToolList />
```

`ToolDatabase` stores tool definitions on disk:

```python
from cam_slicer.tool_database import ToolDatabase

db = ToolDatabase()
db.add_tool("3mm endmill", 3.0, "flat", 12000)
db.save("tools.json")
```

### Tool and Material Library

`ToolMaterialDB` stores tools, materials and recommended parameters. It can also
sync the library to a simple cloud endpoint.

```python
from cam_slicer.tool_material_db import ToolMaterialDB

lib = ToolMaterialDB()
lib.add_tool("1mm vbit", 1.0, "engraver", 15000)
lib.add_material("MDF")
lib.set_params("1mm vbit", "MDF", rpm=10000, feedrate=300.0)
lib.save("library.json")
# lib.sync_to_cloud("https://example.com/upload", token="abcd")
```

`HeightmapViewer` displays a probed heightmap as an interactive 3D surface.

`WorkspaceScanner` ties the debris detection and probing functions together. Use
it in your React UI to start camera, probe or hybrid scans and overlay the
results in `Viewport3D`.

```jsx
import HeightmapViewer from './web/HeightmapViewer';

<HeightmapViewer />
```

`Viewport3D` provides a 3D workspace grid with orbit controls using **react-three-fiber**. Drop STL, OBJ, SVG, DXF or G-code files onto the canvas and they appear centered in the scene. Each file is listed in an overlay with dimensions and a select button. Selected models can be moved or rotated using on-canvas transform controls. All actions are logged to `logs/central.log`.

```jsx
import Viewport3D from './web/Viewport3D';

<Viewport3D />
```

Dropping a G-code file renders its toolpaths in distinct colors. Toggle each
toolhead type in the overlay and press **Play** to animate the toolhead along
the path.

A slider below the viewport steps through layers or segments. Use the dropdown to
switch between **layer** and **segment** mode. The current index is displayed beside
  the slider. Optional **Diagnostics** markers highlight sharp corners or
  potential collisions. An additional **AI Diagnostics** overlay analyses the
  imported models and toolpaths for out-of-bounds moves or deep cuts and places
  color-coded markers. Hover to read a tip and click to focus the camera on the
  problem area.

Snapshot and report export buttons allow saving the current view as PNG or a
PDF report with all diagnostics, the project name and optional notes.

Measurement tools let you click **Dist** or **Angle** and pick points in the scene
to read distances in millimetres or angles in degrees. Notes can be placed at
any position with **Note** and appear as yellow labels on screen. All
actions are logged.

Multiple toolheads can now be visualised at once. Each toolpath segment stores
its **head** type (`spindle`, `laser`, or `picker`) and is drawn in a matching
colour. Small icons mark the start of every path and the active head icon
moves along the path during playback. Use the checkboxes in the sidebar to hide
or show individual heads. The **Optimize Order** button reorders toolpaths with
a nearest-neighbour heuristic and logs the action. Enabling *Material preview*
tints the workpiece showing how milling or laser burning will affect it.

  The rendering is handled by the `ToolpathRenderer` component so new toolheads or styles can be added easily.
`ToolpathSimulator` animates tool moves from a G-code string with play/pause/step controls and live coordinates.

`Viewport3D` also offers a builtâin process simulator. After loading toolpaths, use **Play** to run the machining step by step. A progress bar indicates the current position and you can pause or step through each move. Completed segments are drawn brightly while unfinished moves appear faded. Simulation events are logged and any AI diagnostic at the current step is highlighted. Enable **Load overlay** to see predicted spindle load markers along the path.

```jsx
import ToolpathSimulator from './web/ToolpathSimulator';

const code = 'G1 X0 Y0\nG1 X5 Y0\nG1 X5 Y5';

<ToolpathSimulator gcode={code} />
```

`SurfaceAnalyzer` checks a toolpath against a heightmap and highlights collisions or high segments. Results appear in a small list and are logged.

```jsx
import SurfaceAnalyzer from './web/SurfaceAnalyzer';

const gcode = 'G1 X0 Y0 Z0\nG1 X1 Y0 Z-1';
const map = { points: [ {x:0,y:0,z:0}, {x:1,y:0,z:0} ] };

<SurfaceAnalyzer gcode={gcode} heightmap={map} />
```

`ToolpathOptimizer` runs a mock AI pass using a heightmap and user parameters. The button logs each optimization and shows before/after snippets.

```jsx
import ToolpathOptimizer from './web/ToolpathOptimizer';

const gcode = 'G1 X0 Y0\nG1 X1 Y0';
const heightmap = { points: [{x:0,y:0,z:0},{x:1,y:0,z:-0.1}] };

<ToolpathOptimizer gcode={gcode} heightmap={heightmap} />
`CutStockSimulator` visualizes material removal using voxels and allows exporting the final stock to STL. Events are logged to `logs/central.log`.

```jsx
import CutStockSimulator from "./web/CutStockSimulator";

<CutStockSimulator toolpaths={paths} stock={{x:50,y:50,z:10}} toolRadius={2} />
```

### VR/AR Preview

`VRViewer` allows immersive preview of toolpaths using a VR headset or AR-enabled device. Points are streamed into a WebXR scene and a small head marker animates along the path.

```jsx
import VRViewer from './web/VRViewer';

const points = [ {x:0, y:0, z:0}, {x:50, y:0, z:0}, {x:50, y:50, z:-5} ];

<VRViewer points={points} mode="vr" />
```

Click **Play** to start the simulation. A VR or AR button appears and logs when the mode is enabled.

```jsx
import SetupWizard from './web/SetupWizard';

const gcode = ['G1 X0 Y0', 'G1 X1 Y0'];

<SetupWizard gcodeLines={gcode} />
```

`FeedbackForm` lets beta users quickly send notes that are stored in `logs/central.log`.

```jsx
import FeedbackForm from './web/FeedbackForm';

<FeedbackForm />
```

### Vector Editor
`VectorEditor` now offers node and path editing with layers. Raster images are traced using **OpenCV.js** with adjustable threshold. Choose outline or centerline mode and preview the resulting SVG paths. All edits are logged to `logs/central.log`.
```jsx
import VectorEditor from './web/VectorEditor';

<VectorEditor />
```
### Haptic Toolpath Editor
`HapticToolpathEditor` lets you draw freehand paths on layers and preview them with the simulator. Export visible layers as G-code. All actions are logged to `logs/central.log`.
```jsx
import HapticToolpathEditor from './web/HapticToolpathEditor';

<HapticToolpathEditor />
```



### AI Feedback Advisor

`AIFeedbackAdvisor` analyses a toolpath and recommends feedrate adjustments.
It logs bottleneck locations to `advisor_report.txt`.

```python
from cam_slicer.ai.feedback_advisor import AIFeedbackAdvisor

toolpath = [(0,0,-1), (5,0,-1), (5,5,-5)]
advisor = AIFeedbackAdvisor()
report = advisor.generate_feedback_report(toolpath, feedrate=1200)
print(report)
```
You can also start live monitoring which leverages the vision module to stop
the machine if a hand or obstacle is detected:

```python
advisor.start_live_monitoring()  # runs until you press 'q'
```

The analysis result includes tool and material suggestions and warns about
excessive wear or repeated cuts:

```python
data = advisor.analyze_toolpath(toolpath, feedrate=1200)
print(data["tool_recommendation"])
print(data["material_recommendation"])
```

### Toolpath Prediction and Material Simulation

`ToolpathPredictor` estimates machining time and approximate tool wear, while
`MaterialSimulator` saves a quick preview image of the path. Use
`predict_surface_roughness` to estimate the finish based on feed and tool size.

```python
from cam_slicer.ai.toolpath_predictor import ToolpathPredictor
from cam_slicer.ai.material_simulator import MaterialSimulator
from cam_slicer.ai.roughness_predictor import predict_surface_roughness

tp = [(0, 0, 0), (10, 0, 0), (10, 10, -1)]
pred = ToolpathPredictor()
stats = pred.predict(tp, feedrate=1200)
print(stats["time_sec"], stats["tool_wear"])

sim = MaterialSimulator("aluminum")
sim.simulate(tp, "sim.png")

rough = predict_surface_roughness(feedrate=1200, tool_diameter=6, rpm=10000)
print("Ra", rough)
```

### Force Prediction and Simulation

`ForcePredictor` estimates cutting forces along the toolpath. Use
`simulate_force_profile` to visualize force levels and warn about overloaded
segments.

```python
from cam_slicer.ai.force_prediction import simulate_force_profile

tp = [(0, 0, 0), (20, 0, -1), (20, 20, -1)]
simulate_force_profile(tp, feedrate=1000, depth=1.0, material="steel")
```

### Chip Evacuation and Coolant Management

`ChipCoolantAdvisor` analyzes the toolpath and suggests chip evacuation moves and a coolant setting.

```python
from cam_slicer.ai.chip_coolant import ChipCoolantAdvisor, apply_chip_evacuations

tp = [(0, 0, 0), (60, 0, -2), (120, 0, -2)]
advisor = ChipCoolantAdvisor("aluminum")
info = advisor.analyze(tp, feedrate=800)
tp = apply_chip_evacuations(tp, info["evac_moves"])
print("Coolant:", info["coolant"])  # air blast
```

### Hybrid Operation Assignment

`assign_hybrid_operations` maps vector paths to cutting operations and assigns
the correct tool head for each segment. A summary counts how many segments use
each tool.

```python
from cam_slicer.utils import assign_hybrid_operations

paths = [
    [(0, 0), (1, 0)],
    [(1, 0), (1, 1)],
    [(2, 0), (2, 1)],
]
ops = {0: "cut", 1: "engrave", 2: "drill"}
segments, summary = assign_hybrid_operations(paths, ["router", "laser", "engraver"], ops)
print(summary)
```
All assignments are logged to `logs/central.log`.

### Multi-Head Job Scheduling

Use `prepare_hybrid_job` to assign heads and schedule segments automatically or
call `schedule_multi_head_job` directly when you already have the head
assignment prepared. Provide macros for each head to insert the correct tool
change commands.

```python
from cam_slicer.utils import prepare_hybrid_job
from cam_slicer.core.header_footer import ControllerConfig

cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
macros = {"router": "TOOLCHANGE_router", "laser": "TOOLCHANGE_laser"}
gcode, summary = prepare_hybrid_job(
    paths,
    cfg,
    ops,
    ["router", "laser"],
    head_macros=macros,
)
```

### Hybrid Additive/Subtractive Workflow

Combine 3D printing and milling paths into a single job:

```python
from cam_slicer.utils import blend_additive_subtractive
from cam_slicer.core.header_footer import ControllerConfig

cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
print_paths = [[(0, 0, 0), (10, 0, 0)]]
mill_paths = [[(0, 0, -1), (10, 0, -1)]]
macros = {
    "print_head": "TOOLCHANGE_print_head",
    "router": "TOOLCHANGE_router",
}
gcode, summary = blend_additive_subtractive(
    print_paths, mill_paths, cfg, head_macros=macros
)
```

### Hybrid Laser-Mill Workflow

```python
from cam_slicer.utils import blend_laser_mill
from cam_slicer.core.header_footer import ControllerConfig

cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
laser = [[(0, 0, 0), (5, 0, 0)]]
mill = [[(0, 0, -1), (5, 0, -1)]]
macros = {"laser": "TOOLCHANGE_laser", "router": "TOOLCHANGE_router"}
gcode, summary = blend_laser_mill(
    laser,
    mill,
    cfg,
    laser_power=500.0,
    feedrate_laser=1200.0,
    feedrate_mill=800.0,
    head_macros=macros,
)
```

### Operation Assigner (React)

Use the `OperationAssigner` component to pick an operation (Cut, Engrave, Mill, Print) and tool head (Laser, Spindle, Print Head) for each imported path. In `CuraSlicerUI` this appears under **Tools & Operations** and the assignments are shown in the **Hybrid Operations** preview. A summary count of heads and operations is displayed.

```jsx
import OperationAssigner from "./web/OperationAssigner";

const paths = [[{x:0,y:0},{x:1,y:0}], [{x:0,y:1},{x:1,y:1}]];

<OperationAssigner paths={paths} />
```

The right panel also offers an **Adaptive Toolpath** checkbox. When enabled the
generated paths try to follow the scanned surface using experimental logic.



### Function Scanner

`function_scanner.py` recursively lists all top level functions in this repository and prints the results as JSON to the console.

```bash
python function_scanner.py [directory]
```

The script writes progress to `logs/central.log` and is covered by simple tests in `tests/test_function_scanner.py`.

### Function Inventory

A full list of all detected top-level functions in this repository is stored in `functions_list.json`. Run the scanner to regenerate:

```bash
python function_scanner.py > functions_list.json
```

Each entry has the file path, function name, and first line of the docstring.

### Project Export

Generate a snapshot of the entire repository as `export_codex.json`:

```bash
python project_exporter.py --root . --output export_codex.json
```

The resulting JSON list stores every file with its checksum and a final
`build_info` block showing dependencies and entry points.

### Cura Style Web UI

A React component `CuraSlicerUI` offers a dark themed interface similar to Cura.
It logs menu selections and import/export actions to `logs/central.log`. The sidebar
includes groups like *AI Analyze & Optimize*, *Experimental features* and *Diagnostics* alongside *Export & Streaming* and *Monitoring*.
The AI panel lists plugins and provides analysis or optimization buttons. Diagnostics offers
actions to run the project tests and generate a system report. The Experimental section contains toggles such as
**Adaptive Toolpath** so you can try new capabilities. See `web/README.md` for full usage.
Advanced users can open the **Experimental AI** panel to combine SurfaceAnalyzer, ToolpathOptimizer and early AI Feedback tools.
You can drop design files onto the canvas to preview and transform them before machining.

## Setup Wizard
Run the onboarding wizard to configure a new project:

```bash
python -m cam_slicer.cli_wizard --lang en --project myproj --profile MACHINE_GRBL_SMALL
```

The wizard guides you through language selection, project folder creation and machine profile choice. Results are logged to `logs/central.log`.

## Diagnostics CLI
Use the diagnostics tool to export logs, export only warnings and errors, or generate a system report:

```bash
python -m cam_slicer.cli_diagnostics export-logs mylog.txt
python -m cam_slicer.cli_diagnostics export-critical critical.log --level ERROR --email ops@example.com
python -m cam_slicer.cli_diagnostics system-report report.txt
```

## REST API
The FastAPI server exposes core features over HTTP. Start it with:

```bash
python -m cam_slicer.api_server
```

Endpoints include `/plugins` to list plugins and `/plugins/{name}` to run one. `/export` converts a toolpath to G-code and `/optimize` optimizes it. Browse `http://localhost:8000/docs` for full API documentation.

## Robotic Arm Interface
The slicer can generate extended G-code for 6-axis robots or issue `MOVE_ARM` commands with direct joint angles. A kinematic profile describes the robot tool centre point (TCP), joint limits and workspace.

### Kinematic Profile Format
A JSON profile contains:
```json
{
  "name": "6axis_basic",
  "link_lengths": [100, 100],
  "joint_types": ["revolute", "revolute"],
  "tcp": [0, 0, 0, 0, 0, 0],
  "joint_limits": [[-180, 180], [-90, 90], [-135, 135], [-180, 180], [-120, 120], [-360, 360]],
  "workspace": [[-500, 500], [-500, 500], [0, 1000]]
}
```
Load or save profiles with `ArmKinematicProfile.load()` and `profile.save()`.
`load()` accepts JSON or YAML files.

### Kinematics
`forward_kinematics(joints)` returns the tool position while
`inverse_kinematics(pose)` solves joint angles for planar 2âjoint arms.
Use `workspace_to_joints()` to check workspace limits and get angles in one call.
`check_collision(joints, fence)` verifies that no joint enters forbidden zones defined by `GeoFence`.

### Extended G-code
Use `format_extended_g1()` to produce lines like `G1 X10 Y0 Z50 A0 B90 C0 F1500`. For joint-space commands call `format_move_arm()` which outputs `MOVE_ARM J1=... J2=...`.

### Multi-axis CNC Support
Toolpaths may include A/B/C rotations. The viewer imports such files and animates tool orientation. `toolpath_to_gcode` writes these angles so controllers like Fanuc, Siemens, Haas and LinuxCNC can run the code without extra postprocessing.

### Exporting Toolpaths
`export_robotic_toolpath(toolpath, profile, mode)` converts a list of XYZABC points to either extended G-code or `MOVE_ARM` lines. Existing CNC code remains compatible because the XYZ axes follow standard conventions and A/B/C are optional.

Use `export_toolpath(toolpath, profile, controller_cfg, robot_mode=True)` to switch
between standard CNC output and robotic commands. When `robot_mode=False` the
toolpath is passed to `toolpath_to_gcode` so existing post processors remain
functional. Provide `mode="joint"` to emit `MOVE_ARM` lines or keep the default
`"xyzabc"` for extended G1 commands. Additional post processors can subclass
`PostProcessor` for vendor-specific formats.

### Toolpath Conversion Pipeline
`convert_xyz_toolpath(toolpath, profile)` takes a list of standard XYZ points and
returns robot commands while checking joint limits and collisions. Use
`batch_convert([path1, path2], profile)` to process multiple toolpaths or
`stream_converted_toolpath(toolpath, profile, sender)` to send commands in real
time. Warnings are logged to `logs/central.log` if a pose is unreachable or
dangerous.

Use `stream_robotic_toolpath(toolpath, profile, '/dev/ttyUSB0')` to export the
commands and stream them to the controller in one step.

### AI Trajectory Optimization
`optimize_robotic_trajectory(toolpath, profile)` analyzes each pose for joint
limits, high acceleration or singularities and returns a smoothed toolpath with
warnings for risky moves. The FastAPI server exposes `/robot_optimize` to call
this optimizer from the React UI.

### Automatic Base Position Optimization
`plan_base_zones(toolpath, profile, (0,1000), (0,1000))` searches for the best
portal position so every pose stays within the robot workspace. The toolpath is
split into zones when necessary and each zone includes a base `(x, y)` for the
gantry. Use `optimize_base_position` if you only need a single position or
`optimize_base_position_ai` to consider fixtures and debris via a `GeoFence`.

### Workspace Zoning and Job Splitting
`plan_workspace_zones(toolpath, profile, (0,1000), (0,1000))` builds on the base
planner and inserts collisionâchecked moves to reposition the gantry. Each zone
contains the base move commands and any warnings if an unsafe path is detected.
Review the returned `ZonePlan` list in the UI so the operator can confirm all
transitions before starting the job.

```python
zones = plan_workspace_zones(toolpath, profile, (0,1000), (0,1000))
for z in zones:
    print(z.base, len(z.toolpath), z.move_cmds, z.warnings)
```
After dropping this JSON zone plan into the viewport, **ZoneVisualizer** shows
each base position and animates transitions so you can verify reachable areas or
blind spots. Drag the gray cubes to tweak base locations and replay the
simulation until all zones look correct.

### 3+2 Indexed Positioning
`generate_indexed_gcode(toolpath, [(0,0,0), (90,0,0)], cfg)` rotates the part
between segments so complex sides can be machined without a full 5âaxis path.
Each orientation is issued with a `G0 A.. B.. C..` move before exporting the
segment. Use `segment_length` to control how many points belong to each setup.

### Performance and Parallel Execution
Heavy computations like G-code export are decorated with `profiled` to log runtime
and memory usage. Set `CAM_PERF_THRESHOLD` to adjust the warning threshold.
Enable parallel processing of large toolpaths:

```python
from cam_slicer.core.gcode_export import toolpath_to_gcode, ControllerConfig
cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
lines = toolpath_to_gcode(path, cfg, parallel=True, workers=4)
```

Performance warnings appear in `logs/central.log`.
## Universal Post-Processor
The library automatically detects the connected controller firmware and
applies a matching postâprocessor plugin. Built-in processors support **GRBL**,
**Marlin** and **Smoothieware**. Custom processors can be placed in
`cam_slicer/post_processors` and loaded with `load_post_processors()`.

Example:
```python
from cam_slicer.post_processors import auto_post_process, load_post_processors
load_post_processors()
with open('job.gcode') as f:
    lines = f.read().splitlines()
processed = auto_post_process(lines, '/dev/ttyUSB0')
```


## Custom Strategy Scripting
Advanced users can write Python scripts to implement new toolpath strategies. Each script must define a `register()` function returning a `Strategy` instance.

Example `my_strategy.py`:
```python
from cam_slicer.utils.custom_strategy import Strategy

def register():
    def apply(tp, params):
        # simple demo reverses toolpath
        return list(reversed(tp))
    return Strategy(name="reverse_demo", description="demo", apply=apply)
```

Load and run the strategy:
```python
from cam_slicer.utils import load_strategy, run_strategy
strategy = load_strategy("my_strategy.py")
result = run_strategy(toolpath, strategy, depth=-1)
```
All actions are logged to `logs/central.log`.

