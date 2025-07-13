import os, sys, types
sys.path.insert(0, os.path.abspath('.'))

from cam_slicer.cli_ros_bridge import main

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
        return True
    def init(self):
        self.ok_flag = True
    def shutdown(self):
        self.ok_flag = False
    def create_node(self, name):
        return self.node
    def spin_once(self, node, timeout_sec=0.1):
        pass


def test_cli_send(monkeypatch):
    """Test sending a command via CLI."""
    dummy = DummyRclpy()
    monkeypatch.setitem(sys.modules, 'rclpy', dummy)
    monkeypatch.setitem(sys.modules, 'rclpy.node', types.SimpleNamespace(Node=DummyNode))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', types.SimpleNamespace(String=types.SimpleNamespace))
    main(['send', '/robot/command', 'start'])
    assert dummy.node.publishers['/robot/command'].published[0] == 'start'

