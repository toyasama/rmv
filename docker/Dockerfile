# Official ROS2 Humble Image with Desktop Support
FROM osrf/ros:humble-desktop

# Python dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    vim \
    build-essential \
    && apt-get clean

# Non-root user for development
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# Additional Python libraries for the project
RUN pip3 install --no-cache-dir \
    numpy \
    matplotlib \
    pyyaml \
    opencv-python-headless

# ROS2 setup to bashrc for the user
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Expose ROS2 ports for communication
EXPOSE 11311
