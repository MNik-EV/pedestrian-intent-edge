"""ROS2 thin wrapper node for vision_perception.

Algorithm logic lives in amp_core; this node handles topics/params/lifecycle.
On platforms without hardware, prefer scripts/run_mock_stack.py.
"""

from __future__ import annotations


def main() -> None:
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        print("vision_perception: rclpy not available — use PC mock stack.")
        return

    class AmpNode(Node):
        def __init__(self) -> None:
            super().__init__("vision_perception")
            self.declare_parameter("enabled", True)
            self.get_logger().info("vision_perception started (amp_core-backed)")

    rclpy.init()
    node = AmpNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
