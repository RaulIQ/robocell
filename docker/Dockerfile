FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rviz2 ros-jazzy-robot-state-publisher ros-jazzy-xacro \
      ros-jazzy-cv-bridge ros-jazzy-rmw-cyclonedds-cpp \
      python3-pip python3-venv python3-opencv python3-numpy python3-colcon-common-extensions python3-tk \
      can-utils socat iproute2 gh \
      libgl1 libegl1 libglfw3 libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
RUN install -d -o ubuntu -g ubuntu /opt/venv
USER ubuntu

RUN python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir --upgrade pip
ENV PATH=/opt/venv/bin:$PATH

RUN pip install --no-cache-dir \
      "numpy<2" mujoco==3.16.0 matplotlib==3.10.8 pymodbus pyserial python-can canopen transitions py_trees

COPY pyproject.toml /opt/robocell/pyproject.toml
RUN pip install --no-cache-dir --group /opt/robocell/pyproject.toml:dev
RUN echo /ws/typings > "$(python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')/robocell-typings.pth"
RUN echo /ws/src/robocell_sim > "$(python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')/robocell-sim.pth"

WORKDIR /ws
