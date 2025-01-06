# Official ROS2 Humble
FROM osrf/ros:humble-desktop

# Python dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && apt-get clean

# no root user
RUN useradd -ms /bin/bash devuser
USER devuser
WORKDIR /home/devuser

# python libraries
RUN pip3 install --no-cache-dir \
    numpy \
    matplotlib \
    pyyaml

# Configuration of ROS 2 for user
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

