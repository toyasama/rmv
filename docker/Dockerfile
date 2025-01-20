# Official ROS2 Humble Image with Desktop Support
FROM osrf/ros:humble-desktop

# Python dependencies
RUN rm -rf /var/lib/apt/lists/* && apt-get update

RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    build-essential \
    && apt-get clean

RUN pip3 install git+https://github.com/DLu/tf_transformations.git

RUN pip3 install --no-cache-dir \
    numpy \
    matplotlib \
    pyyaml \
    opencv-python-headless \
    transforms3d

# Non-root user for development
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=1000

# Additional Python libraries for the project
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

RUN echo 'HISTFILE=~/.bash_history' >> /home/devuser/.bashrc \
    && echo 'HISTSIZE=1000' >> /home/devuser/.bashrc \
    && echo 'HISTFILESIZE=2000' >> /home/devuser/.bashrc \
    && echo 'shopt -s histappend' >> /home/devuser/.bashrc \
    && echo 'PROMPT_COMMAND="history -a; $PROMPT_COMMAND"' >> /home/devuser/.bashrc

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo ". /home/devuser/workspace/install/setup.bash" >> /home/devuser/.bashrc

# Expose ROS2 ports for communication
EXPOSE 11311
