# PAC 2026 AHEAD development image: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, MoveIt 2.
# Build and run with docker/run.sh (mounts this repository at /ws/pac2026).
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    IGNITION_VERSION=fortress \
    GZ_VERSION=fortress

# ROS packages the launch files and the Hyundai packages need on top of desktop-full
# (desktop-full already has ros_gz / Gazebo Fortress, xacro, rviz2).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-moveit \
      ros-humble-moveit-visual-tools \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-ros2controlcli \
      ros-humble-gz-ros2-control \
      ros-humble-joint-state-publisher-gui \
      python3-colcon-common-extensions \
      python3-pip \
      python3-pytest \
      python3-networkx \
      python3-opencv \
      nlohmann-json3-dev \
      libcurl4-openssl-dev \
      libboost-system-dev \
      liburdfdom-tools \
      git \
    && rm -rf /var/lib/apt/lists/*

# Python packages of pyproject.toml that Ubuntu 22.04 ships too old (Shapely 1.8) or not at all.
RUN pip3 install --no-cache-dir "Shapely>=2.0,<3" "pybullet>=3.2.6,<4" "aiohttp>=3.8,<4" "Pillow>=10,<13"

# Anything a package.xml declares that is still missing (keeps the image in step with the repo).
COPY ros2_ws/src /tmp/pac_src
COPY external /tmp/pac_external
RUN apt-get update && (rosdep init 2>/dev/null || true) && rosdep update --rosdistro humble && \
    rosdep install --from-paths /tmp/pac_src /tmp/pac_external --ignore-src -r -y --rosdistro humble && \
    rm -rf /var/lib/apt/lists/* /tmp/pac_src /tmp/pac_external

ENV PAC_REPO=/ws/pac2026
WORKDIR /ws/pac2026
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/pac2026/scripts/env.sh ] && source /ws/pac2026/scripts/env.sh' >> /root/.bashrc
CMD ["bash"]
