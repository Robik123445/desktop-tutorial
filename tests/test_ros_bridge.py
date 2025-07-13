import types
import sys
import json

class DummyPublisher:
    def __init__(self):
        self.published = []
    def publish(self, msg):
        self.published.append(msg.data)

class DummyNode:
    def __init__(self):
        self.publishers = {}
        self.sub_callbacks = {}
    def create_publisher(self, _type, topic, _q):
        pub = DummyPublisher()
        self.publishers[topic] = pub
        return pub
    def create_subscription(self, _type, topic, cb, _q):
        self.sub_callbacks[topic] = cb
    def destroy_node(self):
        pass

class DummyRclpy:
    def __init__(self):
        self.node = DummyNode()
        self.ok_flag = False
    def ok(self):
        return self.ok_flag
    def init(self):
        self.ok_flag = True
    def shutdown(self):
        self.ok_flag = False
    def create_node(self, name):
        return self.node
    def spin_once(self, node, timeout_sec=0.1):
        pass


def test_ros_bridge_publish(monkeypatch):
    """Ensure ROSBridge publishes JSON position data."""
    dummy = DummyRclpy()
    monkeypatch.setitem(sys.modules, 'rclpy', dummy)
    monkeypatch.setitem(sys.modules, 'rclpy.node', types.SimpleNamespace(Node=DummyNode))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', types.SimpleNamespace(String=types.SimpleNamespace))
    import importlib
    bridge_mod = importlib.import_module('cam_slicer.ros_bridge')
    importlib.reload(bridge_mod)
    bridge = bridge_mod.ROSBridge()
    bridge.publish_position(1, 2, 3)
    assert json.loads(dummy.node.publishers['/robot/position'].published[0]) == {'x': 1, 'y': 2, 'z': 3}
    # simulate command callback
    dummy.node.sub_callbacks['/robot/command'](types.SimpleNamespace(data='start'))
    # no exception means callback executed


def test_ros_bridge_feedback(monkeypatch):
    """Ensure feedback and error messages are published."""
    dummy = DummyRclpy()
    monkeypatch.setitem(sys.modules, 'rclpy', dummy)
    monkeypatch.setitem(sys.modules, 'rclpy.node', types.SimpleNamespace(Node=DummyNode))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', types.SimpleNamespace(String=types.SimpleNamespace))
    import importlib
    bridge_mod = importlib.import_module('cam_slicer.ros_bridge')
    importlib.reload(bridge_mod)
    bridge = bridge_mod.ROSBridge()
    bridge.publish_feedback('ok')
    bridge.publish_error('e')
    assert dummy.node.publishers['/robot/feedback'].published[0] == 'ok'
    assert dummy.node.publishers['/robot/error'].published[0] == 'e'


def test_ros_bridge_extra_topics(monkeypatch):
    """Heartbeat, warning and twin state are published."""
    dummy = DummyRclpy()
    monkeypatch.setitem(sys.modules, 'rclpy', dummy)
    monkeypatch.setitem(sys.modules, 'rclpy.node', types.SimpleNamespace(Node=DummyNode))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', types.SimpleNamespace(String=types.SimpleNamespace))
    import importlib
    bridge_mod = importlib.import_module('cam_slicer.ros_bridge')
    importlib.reload(bridge_mod)
    bridge = bridge_mod.ROSBridge()
    bridge.publish_heartbeat('alive')
    bridge.publish_warning('hot')
    class DT:
        def __init__(self):
            self.cbs = []
        def add_listener(self, cb):
            self.cbs.append(cb)
    twin = DT()
    bridge.attach_digital_twin(twin)
    twin.cbs[0]({'x': 1})
    assert dummy.node.publishers['/robot/heartbeat'].published[0] == 'alive'
    assert dummy.node.publishers['/robot/warning'].published[0] == 'hot'
    assert json.loads(dummy.node.publishers['/robot/twin'].published[0]) == {'x': 1}


