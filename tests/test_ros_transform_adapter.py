import types
import sys
import math
from cam_slicer.ros_transform_adapter import (
    transform_config_to_pose,
    toolpath_to_pose_list,
    compose_transforms,
    invert_transform,
    map_point_between_frames,
    gcode_to_pose_list,
)
from cam_slicer.robotics.kinematics import TransformConfig


def test_transform_config_to_pose():
    """Convert TransformConfig to pose dict."""
    cfg = TransformConfig(rotation_deg=90, offset=(1, 2, 3))
    pose = transform_config_to_pose(cfg)
    assert pose["position"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert math.isclose(pose["orientation"]["w"], math.cos(math.radians(90)/2), rel_tol=1e-6)


def test_toolpath_to_pose_list():
    """Toolpath is converted to list of poses."""
    path = [(0, 0, 0), (1, 0, 0)]
    poses = toolpath_to_pose_list(path, TransformConfig())
    assert len(poses) == 2
    assert poses[1]["position"]["x"] == 1


def test_compose_and_invert():
    """Compose transforms and invert back to original."""
    cfg1 = TransformConfig(offset=(1, 0, 0), rotation_deg=90)
    cfg2 = TransformConfig(offset=(0, 2, 0))
    composed = compose_transforms(cfg1, cfg2)
    inv = invert_transform(composed)
    orig = compose_transforms(composed, inv)
    assert math.isclose(orig.offset[0], 0.0, abs_tol=1e-6)
    assert math.isclose(orig.offset[1], 0.0, abs_tol=1e-6)


def test_map_point_between_frames():
    """Point mapping between frames uses transforms."""
    src = TransformConfig(offset=(1, 0, 0))
    dst = TransformConfig(offset=(0, 1, 0))
    pt = map_point_between_frames((0, 0, 0), src, dst)
    assert pt == (1.0, -1.0, 0.0)


def test_gcode_to_pose_list():
    """Parse G-code into poses."""
    lines = ["G1 X1 Y0 Z0", "G1 X1 Y1 Z0"]
    poses = gcode_to_pose_list(lines)
    assert len(poses) == 2


class DummyBroadcaster:
    def __init__(self):
        self.sent = []
    def sendTransform(self, msg):
        self.sent.append(msg)


class DummyBuffer:
    def lookup_transform(self, target, source, time):
        tf = types.SimpleNamespace()
        tf.transform = types.SimpleNamespace(
            translation=types.SimpleNamespace(x=1, y=2, z=3),
            rotation=types.SimpleNamespace(x=0, y=0, z=0, w=1),
        )
        return tf


def test_ros_adapter_broadcast(monkeypatch):
    """Adapter broadcasts transform via tf2."""
    dummy_rclpy = types.SimpleNamespace(
        ok=lambda: True,
        init=lambda: None,
        shutdown=lambda: None,
        create_node=lambda name: types.SimpleNamespace(
            get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None)),
            destroy_node=lambda: None,
        ),
        spin_once=lambda n, timeout_sec=0.1: None,
        time=types.SimpleNamespace(Time=lambda: None),
    )
    monkeypatch.setitem(sys.modules, 'rclpy', dummy_rclpy)
    monkeypatch.setitem(sys.modules, 'tf2_ros', types.SimpleNamespace(
        TransformBroadcaster=lambda node: DummyBroadcaster(),
        Buffer=lambda: DummyBuffer(),
        TransformListener=lambda buf, node: None,
    ))
    import cam_slicer.ros_transform_adapter as mod
    monkeypatch.setattr(mod, 'rclpy', dummy_rclpy)
    class DummyTS:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id="", child_frame_id="")
            self.transform = types.SimpleNamespace(
                translation=types.SimpleNamespace(x=0, y=0, z=0),
                rotation=types.SimpleNamespace(x=0, y=0, z=0, w=1),
            )

    monkeypatch.setattr(mod, 'TransformBroadcaster', lambda node: DummyBroadcaster())
    monkeypatch.setattr(mod, 'Buffer', lambda: DummyBuffer())
    monkeypatch.setattr(mod, 'TransformListener', lambda buf, node: None)
    monkeypatch.setattr(mod, 'TransformStamped', DummyTS)
    from cam_slicer.ros_transform_adapter import ROSTransformAdapter
    adapter = ROSTransformAdapter()
    cfg = TransformConfig(offset=(0,0,0))
    adapter.broadcast_transform(cfg)
    assert isinstance(adapter.broadcaster.sent[0], DummyTS)


