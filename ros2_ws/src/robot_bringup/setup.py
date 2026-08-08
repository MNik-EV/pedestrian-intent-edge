from setuptools import setup

package_name = "robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/placeholder.launch.py", "launch/robot.launch.py"],
        ),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AMP Research",
    maintainer_email="research@amp-robot.local",
    description="AMP robot bringup",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "robot_bringup_node = robot_bringup.node:main",
        ],
    },
)
