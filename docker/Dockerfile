FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rviz2 ros-jazzy-robot-state-publisher ros-jazzy-xacro \
      ros-jazzy-cv-bridge ros-jazzy-rmw-cyclonedds-cpp \
      python3-pip python3-opencv python3-numpy python3-colcon-common-extensions \
      can-utils socat iproute2 \
      libgl1 libegl1 libglfw3 libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir --break-system-packages \
      "numpy<2" mujoco pymodbus pyserial python-can canopen transitions py_trees

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
WORKDIR /ws
