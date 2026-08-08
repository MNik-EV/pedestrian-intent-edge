"""ROS2 thin wrapper node for simulation.

Algorithm logic lives in amp_core; this node handles topics/params/lifecycle.
On platforms without hardware, prefer scripts/run_mock_stack.py.
"""

from __future__ import annotations


def main() -> None:
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        print("simulation: rclpy not available — use PC mock stack.")
        return

    class AmpNode(Node):
        def __init__(self) -> None:
            super().__init__("simulation")
            self.declare_parameter("enabled", True)
            self.get_logger().info("simulation started (amp_core-backed)")

    rclpy.init()
    node = AmpNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
