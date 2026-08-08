# Optional Docker (Ubuntu 24.04 + ROS2 Jazzy) for PC builds when Docker is available.
FROM ros:jazzy-ros-base

WORKDIR /ws
COPY requirements.txt /ws/requirements.txt
RUN apt-get update && apt-get install -y python3-pip && pip3 install -r /ws/requirements.txt

COPY . /ws/amp
WORKDIR /ws/amp/ros2_ws
# Build when image is used on Linux hosts:
# RUN bash -lc "source /opt/ros/jazzy/setup.bash && colcon build"

CMD ["bash"]
