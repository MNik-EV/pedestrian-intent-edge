"""ROS2 thin wrapper node for lidar_perception.

Algorithm logic lives in amp_core; this node handles topics/params/lifecycle.
On platforms without hardware, prefer scripts/run_mock_stack.py.
"""

from __future__ import annotations


def main() -> None:
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        print("lidar_perception: rclpy not available — use PC mock stack.")
        return

    class AmpNode(Node):
        def __init__(self) -> None:
            super().__init__("lidar_perception")
            self.declare_parameter("enabled", True)
            self.get_logger().info("lidar_perception started (amp_core-backed)")

    rclpy.init()
    node = AmpNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
