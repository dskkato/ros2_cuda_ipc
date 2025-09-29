# syntax=docker/dockerfile:1

ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install build tools, ROS 2 development dependencies, CUDA toolkit, and utilities
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        clang \
        cmake \
        git \
        gdb \
        lsb-release \
        ninja-build \
        pkg-config \
        python3-colcon-common-extensions \
        python3-pip \
        python3-vcstool \
        python3-rosdep \
        python3-rosinstall-generator \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-ros-base \
        wget \
        curl \
        sudo \
        nvidia-cuda-toolkit \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (safe to rerun if already initialized)
RUN rosdep update || rosdep init && rosdep update

# Upgrade pip and install Python development helpers
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir \
        pre-commit \
        vcstool

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Source ROS 2 setup in every terminal session
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/skel/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

ENV ROS_DISTRO=${ROS_DISTRO}

