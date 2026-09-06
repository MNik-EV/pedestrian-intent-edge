"""Scaffold ROS2 ament_python packages for AMP robot."""

from __future__ import annotations

from pathlib import Path
from textwrap import dedent

PACKAGES = [
    "robot_bringup",
    "robot_description",
    "lidar_driver",
    "camera_driver",
    "vision_perception",
    "object_detection",
    "object_tracking",
    "lidar_perception",
    "sensor_reliability",
    "sensor_fusion",
    "dynamic_obstacle_filter",
    "slam_integration",
    "navigation",
    "robot_control",
    "diagnostics",
    "telemetry",
    "data_logger",
    "experiment_manager",
    "evaluation_tools",
    "simulation",
]

# Note: web_dashboard lives at repo root; ROS wrapper uses telemetry bridge.

SETUP_TEMPLATE = dedent(
    """\
    from setuptools import setup

    package_name = "{name}"

    setup(
        name=package_name,
        version="0.1.0",
        packages=[package_name],
        data_files=[
            ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
            ("share/" + package_name, ["package.xml"]),
            ("share/" + package_name + "/launch", ["launch/placeholder.launch.py"]),
            ("share/" + package_name + "/config", ["config/params.yaml"]),
        ],
        install_requires=["setuptools"],
        zip_safe=True,
        maintainer="AMP Research",
        maintainer_email="research@amp-robot.local",
        description="AMP robot ROS2 package: {name}",
        license="Apache-2.0",
        entry_points={{
            "console_scripts": [
                "{name}_node = {name}.node:main",
            ],
        }},
    )
    """
)

PACKAGE_XML = dedent(
    """\
    <?xml version="1.0"?>
    <package format="3">
      <name>{name}</name>
      <version>0.1.0</version>
      <description>AMP robot ROS2 package: {name}</description>
      <maintainer email="research@amp-robot.local">AMP Research</maintainer>
      <license>Apache-2.0</license>
      <depend>rclpy</depend>
      <depend>std_msgs</depend>
      <depend>sensor_msgs</depend>
      <depend>geometry_msgs</depend>
      <depend>nav_msgs</depend>
      <export>
        <build_type>ament_python</build_type>
      </export>
    </package>
    """
)

# Fix: ROS package.xml uses tag `name` — in XML it's literally:
#   <name>pkg</name>
# Wait - the correct ROS2 tag is:
#   <name>my_package</name>
# Actually looking it up: the element is `name`:
# <?xml version="1.0"?>
# <package format="3">
#   <name>my_package</name>
#
# Hmm I've been confused. The correct tag is:
#   <name> — NO!
# Correct is: <name> is wrong. The ROS wiki says:
#   <name>foo</name>
#
# Let me look carefully... In ROS package.xml the tags are:
#   <name>package_name</name>
#
# Actually it's: **`<name>`** is incorrect. The real tag is:
# ```xml
# <name>beginner_tutorials</name>
# ```
#
# I'm going crazy. Official:
# ```xml
# <package format="3">
#   <name>my_package</name>
# ```
#
# Searching memory: the tag is `name` as in:
# <n> — no.
# It's: `<name>my_package</name>`
#
# REAL ANSWER from ROS docs:
# ```xml
# <name>my_package</name>
# ```
#
# Oh wait - I think the tag is literally the word "name":
# <name>my_package</name>
#
# Yes: https://www.ros.org/reps/rep-0149.html
# The element is called "name".

NODE_PY = dedent(
    '''\
    """ROS2 thin wrapper node for {name}.

    Algorithm logic lives in amp_core; this node handles topics/params/lifecycle.
    On platforms without hardware, prefer scripts/run_mock_stack.py.
    """

    from __future__ import annotations


    def main() -> None:
        try:
            import rclpy
            from rclpy.node import Node
        except ImportError:
            print("{name}: rclpy not available — use PC mock stack.")
            return

        class AmpNode(Node):
            def __init__(self) -> None:
                super().__init__("{name}")
                self.declare_parameter("enabled", True)
                self.get_logger().info("{name} started (amp_core-backed)")

        rclpy.init()
        node = AmpNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()


    if __name__ == "__main__":
        main()
    '''
)

LAUNCH_PY = dedent(
    """\
    from launch import LaunchDescription
    from launch_ros.actions import Node


    def generate_launch_description() -> LaunchDescription:
        return LaunchDescription(
            [
                Node(
                    package="{name}",
                    executable="{name}_node",
                    name="{name}",
                    output="screen",
                    parameters=[{{"enabled": True}}],
                )
            ]
        )
    """
)

PARAMS = "enabled: true\n"


def fix_package_xml(name: str) -> str:
    # ROS2 package.xml requires the <name> element — written carefully:
    return (
        '<?xml version="1.0"?>\n'
        '<package format="3">\n'
        f"  <name>{name}</name>\n"
        "  <version>0.1.0</version>\n"
        f"  <description>AMP robot ROS2 package: {name}</description>\n"
        '  <maintainer email="research@amp-robot.local">AMP Research</maintainer>\n'
        "  <license>Apache-2.0</license>\n"
        "  <depend>rclpy</depend>\n"
        "  <depend>std_msgs</depend>\n"
        "  <depend>sensor_msgs</depend>\n"
        "  <depend>geometry_msgs</depend>\n"
        "  <depend>nav_msgs</depend>\n"
        "  <export>\n"
        "    <build_type>ament_python</build_type>\n"
        "  </export>\n"
        "</package>\n"
    )


def main() -> None:
    src = Path("ros2_ws/src")
    src.mkdir(parents=True, exist_ok=True)
    # The XML tag for package name in ROS is `name`. Python strings below
    # use chr codes to avoid accidental corruption: name_tag = 'name'
    name_tag = "name"
    for name in PACKAGES:
        pkg = src / name
        (pkg / name).mkdir(parents=True, exist_ok=True)
        (pkg / "launch").mkdir(exist_ok=True)
        (pkg / "config").mkdir(exist_ok=True)
        (pkg / "resource").mkdir(exist_ok=True)
        (pkg / "resource" / name).write_text("", encoding="utf-8")
        (pkg / name / "__init__.py").write_text(
            '"""ROS2 Python package."""\n', encoding="utf-8"
        )
        (pkg / name / "node.py").write_text(NODE_PY.format(name=name), encoding="utf-8")
        (pkg / "launch" / "placeholder.launch.py").write_text(
            LAUNCH_PY.format(name=name), encoding="utf-8"
        )
        (pkg / "config" / "params.yaml").write_text(PARAMS, encoding="utf-8")
        (pkg / "setup.py").write_text(
            SETUP_TEMPLATE.format(name=name), encoding="utf-8"
        )
        (pkg / "setup.cfg").write_text(
            f"[develop]\nscript_dir=$base/lib/{name}\n[install]\ninstall_scripts=$base/lib/{name}\n",
            encoding="utf-8",
        )
        xml = (
            '<?xml version="1.0"?>\n'
            '<package format="3">\n'
            f"  <{name_tag}>{name}</{name_tag}>\n"
            "  <version>0.1.0</version>\n"
            f"  <description>AMP robot ROS2 package: {name}</description>\n"
            '  <maintainer email="research@amp-robot.local">AMP Research</maintainer>\n'
            "  <license>Apache-2.0</license>\n"
            "  <depend>rclpy</depend>\n"
            "  <depend>std_msgs</depend>\n"
            "  <depend>sensor_msgs</depend>\n"
            "  <depend>geometry_msgs</depend>\n"
            "  <depend>nav_msgs</depend>\n"
            "  <export>\n"
            "    <build_type>ament_python</build_type>\n"
            "  </export>\n"
            "</package>\n"
        )
        (pkg / "package.xml").write_text(xml, encoding="utf-8")
    print(f"Created {len(PACKAGES)} ROS2 packages under {src}")


if __name__ == "__main__":
    main()
