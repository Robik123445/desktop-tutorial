"""Minimal ROS2 test node for CAM Slicer integration."""

from __future__ import annotations

import argparse
import logging
from typing import Optional

from .logging_config import setup_logging

setup_logging()

try:  # optional dependency
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except Exception:  # pragma: no cover - ROS2 not installed
    rclpy = None
    Node = object  # type: ignore
    String = object  # type: ignore


class ROSTestNode:
    """Test node subscribing to bridge topics and publishing commands."""

    def __init__(self, node_name: str = "cam_test_node", simulate: bool = False) -> None:
        if rclpy is None:  # pragma: no cover - optional dependency
            raise ImportError("rclpy is required for ROSTestNode")

        self.node_name = node_name
        self.simulate = simulate
        self.node: Optional[Node] = None
        self.logger = logging.getLogger(__name__)
        self.cmd_pub = None
        self.received: list[str] = []
        self._running = False
        self.connect()

    def connect(self) -> None:
        """Initialize ROS node and subscribers/publishers."""
        if not rclpy.ok():
            rclpy.init()
        if self.node:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        self.node = rclpy.create_node(self.node_name)
        self.cmd_pub = self.node.create_publisher(String, "/robot/command", 10)
        topics = [
            "/robot/position",
            "/robot/state",
            "/robot/job",
            "/robot/error",
            "/robot/feedback",
            "/robot/heartbeat",
            "/robot/warning",
            "/robot/twin",
        ]
        for t in topics:
            self.node.create_subscription(String, t, self._on_msg, 10)
        self.logger.info("ROS test node connected")

    def _on_msg(self, msg: String) -> None:
        self.logger.info("Received: %s", msg.data)
        self.received.append(str(msg.data))

    def send_command(self, command: str) -> None:
        """Publish a command to /robot/command."""
        if self.cmd_pub is None:
            return
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.logger.info("Sent command: %s", command)

    def start(self) -> None:
        """Spin ROS events. In simulate mode publish test commands."""
        self._running = True
        while self._running:
            if self.simulate:
                self.send_command("start")
                self.simulate = False  # send once
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as exc:  # pragma: no cover - runtime issue
                self.logger.error("ROS spin error: %s", exc)
                break

    def stop(self) -> None:
        self._running = False
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.logger.info("ROS test node stopped")


def main(argv: Optional[list[str]] = None) -> None:
    """Command line entry point."""
    setup_logging()
    parser = argparse.ArgumentParser(description="Run ROS2 test node")
    parser.add_argument("--simulate", action="store_true", help="Publish mock commands")
    parser.add_argument("--node-name", default="cam_test_node")
    args = parser.parse_args(argv)

    node = ROSTestNode(node_name=args.node_name, simulate=args.simulate)
    try:
        node.start()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()


if __name__ == "__main__":  # pragma: no cover
    main()
