# syntax=docker/dockerfile:1

ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

ENV DEBIAN_FRONTEND=noninteractive

# CUDA and Python binding build dependencies
ARG CUDA_TOOLKIT_VERSION=13-3
ARG CUDA_REPO_KEYRING=cuda-keyring_1.1-1_all.deb

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        wget \
        build-essential \
        cmake \
        pybind11-dev \
        python3-dev \
    && rm -rf /var/lib/apt/lists/*
RUN . /etc/os-release \
    && CUDA_REPO_DISTRO="ubuntu$(printf '%s' "${VERSION_ID}" | tr -d .)" \
    && wget -q \
        "https://developer.download.nvidia.com/compute/cuda/repos/${CUDA_REPO_DISTRO}/x86_64/${CUDA_REPO_KEYRING}" \
    && dpkg -i "${CUDA_REPO_KEYRING}" \
    && rm "${CUDA_REPO_KEYRING}" \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        "cuda-toolkit-${CUDA_TOOLKIT_VERSION}" \
        "cuda-compat-${CUDA_TOOLKIT_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_HOME=/usr/local/cuda
ENV PATH="${CUDA_HOME}/bin:${PATH}"
ENV LD_LIBRARY_PATH="${CUDA_HOME}/compat:${CUDA_HOME}/lib64"

# Initialize rosdep (safe to rerun if already initialized)
RUN rosdep update || (rosdep init && rosdep update)

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Source ROS 2 setup in every terminal session
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/skel/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

ENV ROS_DISTRO=${ROS_DISTRO}
