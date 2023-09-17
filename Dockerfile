FROM osrf/ros:noetic-desktop-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*
    
# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y ros-noetic-gazebo-ros \
    ros-noetic-rviz \
    tmux && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y wget  \
    git \
    python \
    python3-matplotlib \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y ros-noetic-octomap \
    ros-noetic-octomap-ros \
    ros-noetic-octomap-msgs \
    ros-noetic-rqt-gui \
    ros-noetic-rqt-gui-py \
    ros-noetic-joy \
    ros-noetic-gazebo-plugins \
    ros-noetic-tf-conversions \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /robotlar_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/robotlar_ws/src/" >> ~/.bashrc