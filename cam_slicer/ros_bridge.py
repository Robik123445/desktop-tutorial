class ROSBridge:
    """Bidirectional ROS2 communication for CAM/CNC and robots."""

    def __init__(self, node_name: str = "cam_slicer_bridge", use_json: bool = True, heartbeat_interval: float = 5.0):
        """Create bridge and connect to ROS2.

        Parameters
        ----------
        node_name : str
            Name of the ROS2 node.
        use_json : bool
            Publish position as JSON if True, otherwise use geometry messages.
        """

        if rclpy is None:  # pragma: no cover - optional dependency
            raise ImportError("rclpy is required for ROSBridge")

        self.node_name = node_name
        self.use_json = use_json
        self.node: Optional[Node] = None
        self.running = False
        self.heartbeat_interval = heartbeat_interval
        self._last_hb = time.time()
        self._twin = None  # type: Optional[Any]
        self.logger = logging.getLogger(__name__)

        self.command_cb: Optional[Callable[[str], None]] = None
        self.feedback_cb: Optional[Callable[[str], None]] = None

        self.connect()

    def connect(self) -> None:
        """(Re)initialize ROS2 node and publishers/subscribers."""
        if not rclpy.ok():
            rclpy.init()
        if self.node:
            try:
                self.node.destroy_node()
            except Exception:  # pragma: no cover - ignore errors
                pass
        self.node = rclpy.create_node(self.node_name)
        self.position_pub = self.node.create_publisher(String, "/robot/position", 10)
        self.state_pub = self.node.create_publisher(String, "/robot/state", 10)
        self.job_pub = self.node.create_publisher(String, "/robot/job", 10)
        self.error_pub = self.node.create_publisher(String, "/robot/error", 10)
        self.warning_pub = self.node.create_publisher(String, "/robot/warning", 10)
        self.feedback_pub = self.node.create_publisher(String, "/robot/feedback", 10)
        self.heartbeat_pub = self.node.create_publisher(String, "/robot/heartbeat", 10)
        self.twin_pub = self.node.create_publisher(String, "/robot/twin", 10)
        self.command_sub = self.node.create_subscription(String, "/robot/command", self._on_command, 10)
        self.logger.info("ROS bridge connected")

    def start(self) -> None:
        """Process ROS events until ``stop`` is called."""

        self.running = True
        while self.running:
            try:
                now = time.time()
                if now - self._last_hb >= self.heartbeat_interval:
                    self.publish_heartbeat("alive")
                    self._last_hb = now
                if not rclpy.ok():
                    self.logger.warning("ROS not running, reconnecting")
                    self.connect()
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as exc:  # pragma: no cover - runtime issue
                self.logger.error("ROS spin error: %s", exc)
                self.connect()

    def stop(self) -> None:
        """Stop processing and shut down ROS."""
        self.running = False
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.logger.info("ROS bridge stopped")

    def publish_position(self, x: float, y: float, z: float) -> None:
        """Publish robot position to ROS."""
        if self.use_json or Pose is object:
            msg = String()
            msg.data = json.dumps({"x": x, "y": y, "z": z})
        else:  # pragma: no cover - real ROS message
            from geometry_msgs.msg import Pose

            msg = Pose()
            msg.position.x = float(x)
            msg.position.y = float(y)
            msg.position.z = float(z)
        self.position_pub.publish(msg)
        self.logger.info("/robot/position %s", msg.data)

    def publish_state(self, state: str) -> None:
        """Publish tool state to ROS."""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.logger.info("/robot/state %s", state)

    def publish_job_status(self, status: str) -> None:
        """Publish current job status to ROS."""
        msg = String()
        msg.data = status
        self.job_pub.publish(msg)
        self.logger.info("/robot/job %s", status)

    def publish_error(self, message: str) -> None:
        """Publish an error message to ROS."""
        msg = String()
        msg.data = message
        self.error_pub.publish(msg)
        self.logger.info("/robot/error %s", message)

    def publish_feedback(self, message: str) -> None:
        """Publish informational feedback to ROS."""
        msg = String()
        msg.data = message
        self.feedback_pub.publish(msg)
        self.logger.info("/robot/feedback %s", message)

    def publish_warning(self, message: str) -> None:
        """Publish a warning message to ROS."""
        msg = String()
        msg.data = message
        self.warning_pub.publish(msg)
        self.logger.info("/robot/warning %s", message)

    def publish_heartbeat(self, status: str) -> None:
        """Publish heartbeat/status info."""
        msg = String()
        msg.data = status
        self.heartbeat_pub.publish(msg)
        self.logger.info("/robot/heartbeat %s", status)

    def publish_twin_state(self, state: Dict[str, Any]) -> None:
        """Publish digital twin state as JSON."""
        msg = String()
        msg.data = json.dumps(state)
        self.twin_pub.publish(msg)
        self.logger.info("/robot/twin %s", msg.data)

    def attach_digital_twin(self, twin: Any) -> None:
        """Stream ``DigitalTwin`` updates to ROS topics."""
        self._twin = twin
        twin.add_listener(self.publish_twin_state)

    def stream_toolpath_block(self, block: str) -> None:
        """Publish a block of G-code or joint commands."""
        msg = String()
        msg.data = block
        self.job_pub.publish(msg)

    def _on_command(self, msg: String) -> None:
        self.logger.debug("Command: %s", msg.data)
        if self.command_cb:
            self.command_cb(msg.data)
