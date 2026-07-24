# syntax=docker/dockerfile:1

ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

ENV DEBIAN_FRONTEND=noninteractive

# CUDA and Python binding dependencies
ARG CUDA_VERSION=13.3.1
ARG CUDA_INSTALLER=cuda_13.3.1_610.43.02_linux.run
ARG CHECKSUM=7c8d3eca60ee10d2c290bdc045f88f09

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        wget \
        build-essential \
        cmake \
        pybind11-dev \
        python3-dev \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN wget -q \
        "https://developer.download.nvidia.com/compute/cuda/${CUDA_VERSION}/local_installers/${CUDA_INSTALLER}" \
    && echo "${CHECKSUM} ${CUDA_INSTALLER}" \
        | md5sum --check - \
    && sh "${CUDA_INSTALLER}" --silent --toolkit \
    && rm "${CUDA_INSTALLER}"
RUN PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install \
        --no-cache-dir \
        cupy-cuda13x

ENV CUDA_HOME=/usr/local/cuda
ENV PATH="${CUDA_HOME}/bin:${PATH}"
ENV LD_LIBRARY_PATH="${CUDA_HOME}/lib64"

# Initialize rosdep (safe to rerun if already initialized)
RUN rosdep update || (rosdep init && rosdep update)

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Source ROS 2 setup in every terminal session
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/skel/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

ENV ROS_DISTRO=${ROS_DISTRO}
