import os, types, sys, importlib
sys.path.insert(0, os.path.abspath('.'))


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


def test_ros_test_node_publish(monkeypatch):
    """ROS test node publishes commands and receives messages."""
    dummy = DummyRclpy()
    monkeypatch.setitem(sys.modules, 'rclpy', dummy)
    monkeypatch.setitem(sys.modules, 'rclpy.node', types.SimpleNamespace(Node=DummyNode))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', types.SimpleNamespace(String=types.SimpleNamespace))

    mod = importlib.import_module('cam_slicer.ros_test_node')
    importlib.reload(mod)
    node = mod.ROSTestNode()
    node.send_command('start')
    assert dummy.node.publishers['/robot/command'].published[0] == 'start'
    # simulate incoming message
    dummy.node.sub_callbacks['/robot/state'](types.SimpleNamespace(data='idle'))
    dummy.node.sub_callbacks['/robot/heartbeat'](types.SimpleNamespace(data='alive'))
    assert 'idle' in node.received
    assert 'alive' in node.received
