# syntax=docker/dockerfile:1

ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install the CUDA toolkit
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        nvidia-cuda-toolkit \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install pre-commit
RUN pip3 install pre-commit --no-cache-dir

# Initialize rosdep (safe to rerun if already initialized)
RUN rosdep update || (rosdep init && rosdep update)

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Source ROS 2 setup in every terminal session
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/skel/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

ENV ROS_DISTRO=${ROS_DISTRO}
