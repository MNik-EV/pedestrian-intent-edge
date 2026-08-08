from setuptools import setup

package_name = "experiment_manager"

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
    description="AMP robot ROS2 package: experiment_manager",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "experiment_manager_node = experiment_manager.node:main",
        ],
    },
)
