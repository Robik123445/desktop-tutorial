import argparse

from .logging_config import setup_logging


def main(argv=None):
    """Simple CLI wrapper for :class:`ROSBridge`."""
    setup_logging()
    parser = argparse.ArgumentParser(description="ROS2 bridge utilities")
    sub = parser.add_subparsers(dest="cmd", required=True)

    send_p = sub.add_parser("send", help="Send a message to a topic")
    send_p.add_argument("topic")
    send_p.add_argument("message")

    listen_p = sub.add_parser("listen", help="Run event loop")
    listen_p.add_argument("--node", default="cam_slicer_bridge")

    args = parser.parse_args(argv)

    from .ros_bridge import ROSBridge
    node_name = getattr(args, 'node', 'cam_slicer_bridge')

    if args.cmd == "send":
        bridge = ROSBridge(node_name=node_name)
        from std_msgs.msg import String

        pub = bridge.node.create_publisher(String, args.topic, 10)
        msg = String()
        msg.data = args.message
        pub.publish(msg)
        bridge.stop()
    elif args.cmd == "listen":
        bridge = ROSBridge(node_name=node_name)
        try:
            bridge.start()
        except KeyboardInterrupt:
            pass
        finally:
            bridge.stop()


if __name__ == "__main__":  # pragma: no cover
    main()


