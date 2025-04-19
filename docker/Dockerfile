# Official ROS2 Humble Image with Desktop Support
FROM osrf/ros:humble-desktop

# Python dependencies
RUN rm -rf /var/lib/apt/lists/* && apt-get update

RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    build-essential \
    ros-humble-rmw-cyclonedds-cpp \  
    && apt-get clean 


RUN pip3 install git+https://github.com/DLu/tf_transformations.git

COPY requirements.txt /requirements.txt
RUN pip3 install --no-cache-dir -r /requirements.txt

# Kivy installation
ENV USE_SDL2=1
ENV USE_GSTREAMER=1

# Install Kivy, SDL2 and Gstreamer dependencies 
RUN apt-get update -o Acquire::http::Pipeline-Depth=0 && apt-get install -y --no-install-recommends \
    build-essential git python3-dev libmtdev-dev libgles2-mesa libsdl2-dev \
    libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev libportmidi-dev ffmpeg \
    libsm6 libxext6 libxrender1 libgstreamer1.0 gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa \
    gstreamer1.0-pulseaudio xclip xsel && \
    rm -rf /var/lib/apt/lists/*

# Install Kivy library
RUN python3 -m pip install --no-cache-dir --upgrade pip wheel Cython \
    && python3 -m pip install --no-cache-dir setuptools==58.2.0 wheel \
    && python3 -m pip install --no-cache-dir docutils pygments kivy[base] kivy[pygame] kivy[angle_sdl2] \
    && python3 -m pip install --no-cache-dir https://github.com/kivymd/KivyMD/archive/master.zip \
    && python3 -m pip install --no-cache-dir gitpython 

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
RUN sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' ~/.bashrc

RUN echo 'HISTFILE=~/.bash_history' >> /home/devuser/.bashrc \
    && echo 'HISTSIZE=1000' >> /home/devuser/.bashrc \
    && echo 'HISTFILESIZE=2000' >> /home/devuser/.bashrc \
    && echo 'shopt -s histappend' >> /home/devuser/.bashrc \
    && echo 'PROMPT_COMMAND="history -a; $PROMPT_COMMAND"' >> /home/devuser/.bashrc

# Configuration ROS2 et Cyclone DDS
RUN echo "source /opt/ros/humble/setup.bash" >> /home/devuser/.bashrc
RUN echo ". /home/devuser/workspace/install/setup.bash" >> /home/devuser/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/devuser/.bashrc
RUN echo "export CYCLONEDDS_URI=file:///home/devuser/workspace/cyclonedds.xml" >> /home/devuser/.bashrc

COPY entrypoint.sh /entrypoint.sh
USER root 
RUN chmod +x /entrypoint.sh

USER $USERNAME
