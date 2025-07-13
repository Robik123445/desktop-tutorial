# Robotics Utilities

Modules for robotic arm motion planning and command export.
Diagnostics helpers aid with predicting joint load and alerting on
collisions using geofence zones.

``plan_workspace_zones`` can now insert spindle stops between zones and
optionally reorder moves using a simple AI route planner:

```python
zones = plan_workspace_zones(toolpath, profile, (0,1000), (0,1000), ai_optimize=True)
for z in zones:
    for cmd in z.move_cmds:
        send(cmd)
```

Use ``map_singularity_zones`` to visualise dead spots:

```python
danger = map_singularity_zones(profile, (0,1000), (0,1000), step=100)
```

```python
from cam_slicer.robotics.kinematics import TransformConfig
from cam_slicer.robotics.sensor_stream import SensorStream
from cam_slicer.robotics.live_feedback_engine import LiveFeedbackEngine
from cam_slicer.robotics.joint_load_predictor import JointLoadPredictor
from cam_slicer.robotics.collision_alerts import CollisionAlerts
from cam_slicer.robotics.head_selector import HeadSelector
from cam_slicer.robotics.operation_mapper import map_operations
```

stream = SensorStream('/dev/ttyUSB0', callback=print)
stream.start()

engine = LiveFeedbackEngine(profile)
engine.process_pose((0, 0, 0))

predictor = JointLoadPredictor()
loads = predictor.predict_load([10, 20], [30, 15])
alerts = CollisionAlerts(profile, GeoFence([(100, 100, 120, 120)]))
alerts.check_pose([10, 20])

# Select heads and map operations
selector = HeadSelector()
head = selector.select_head("cut", material="plywood")
ops = map_operations(["part.svg", "model.stl"])
