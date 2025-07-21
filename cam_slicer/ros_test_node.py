"""Minimal ROS2 test node for CAM Slicer integration."""

from __future__ import annotations

import argparse
import logging
from typing import Optional

from .logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)

try:  # optional dependency
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except Exception as exc:  # pragma: no cover - ROS2 not installed
    logger.warning("ROS2 packages missing: %s", exc)
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
