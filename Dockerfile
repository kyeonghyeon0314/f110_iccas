# Base image with CUDA support
FROM nvidia/cuda:12.6.1-cudnn-devel-ubuntu22.04

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y \
    python3-dev \
    python3-pip \
    python3-tk \
    nano \
    git \
    unzip \
    wget \
    build-essential \
    autoconf \
    libtool \
    cmake \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble and required packages
RUN apt-get update && \
    apt-get install -y \
    locales \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set environment variables
ENV LANG=en_US.UTF-8
ENV ROS_DISTRO=humble

RUN apt-get update && \
    apt-get install -y \
    ros-humble-robot-localization \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-xacro \
    ros-humble-ackermann-msgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    pulseaudio-utils libasound2 \
    && rm -rf /var/lib/apt/lists/*


# Source ROS2 setup in bashrc for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /f110_ws/install/setup.bash" >> /root/.bashrc

# Open terminal when container starts
ENTRYPOINT ["/bin/bash"]
