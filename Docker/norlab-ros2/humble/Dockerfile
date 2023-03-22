
# /// norlab-ros2/dependencies /////////////////////////////////////////////////////////////////////////////////////////
# This container is the base image for all other norlab-ros2 images: dev, deploy, ...
#
# References:
# - NVIDIA Container Runtime on Jetson: https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson
# - dusty-nv/jetson-containers: https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.noetic
# - ROS noetic installation from source: http://wiki.ros.org/noetic/Installation/Source


# Base image: nvidia linux 4 tegra (L4T) nvidia docker container official image
#   l4t-base: https://ngc.nvidia.com/catalog/containers/nvidia:l4t-base
#   l4t-ros-noetic-pytorch: https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic


# JETPACK & L4T release version: https://developer.nvidia.com/embedded/jetpack-archive and https://www.stereolabs.com/blog/nvidia-jetson-l4t-and-jetpack-support/
# Jetpack 4.6 › tag: r32.6.1
# Jetpack 5.1 › tag: r35.2.1

#ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.6.1
ARG BASE_IMAGE=dustynv/ros:humble-pytorch-l4t-r35.2.1
FROM ${BASE_IMAGE} AS ros-base-image
MAINTAINER luc.coupal.1@ulaval.ca

#ARG DS_ROS_PKG=desktop_full
ARG DS_ROS_PKG=ros_base
# Note: base_image `dustynv/ros:humble-pytorch-l4t-r35.2.1` is ros_base as per https://hub.docker.com/layers/dustynv/ros/humble-pytorch-l4t-r35.2.1/images/sha256-85b6e35baf947074b19efbdd628e3d0d5e32d2563557b493721d9de535463316?context=explore
ENV DS_ROS_PKG=${DS_ROS_PKG}
#ENV ROS_DISTRO=humble

ARG DS_IMAGE_ARCHITECTURE=arm64-l4t
ENV DS_IMAGE_ARCHITECTURE=${DS_IMAGE_ARCHITECTURE}

# skip GUI dialog by setting everything to default
ENV DEBIAN_FRONTEND=noninteractive
ENV DS_DEV_WORKSPACE=$HOME/ros2_ws


#RUN TMP_PV=$(python --version) \
#    && if [[ ${TMP_PV:7:3} == "Python 3.8.10" ]]; then \
#        ENV DS_PYTHON3_VERSION=python3.8 \
#    elif [[ $(python --version) == "x86_64" ]]; then \
#        ENV DS_PYTHON3_VERSION=python3.8 \
#    else \
#      echo -e "${DS_MSG_ERROR} This script does not yet support the $(uname -s) $(uname -m) architecture" \
#      exit 1 \
#    fi


WORKDIR "/"


# install development utilities
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        apt-utils \
        lsb-release \
        cmake \
        build-essential \
        curl \
        wget \
        gnupg2 \
        ca-certificates \
        git \
        libusb-dev \
        usbutils \
        vim \
        tree \
        zip gzip tar unzip \
    && rm -rf /var/lib/apt/lists/*


## ===INstall Python 3 step DONE========================================================================================
# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
WORKDIR "${DS_DEV_WORKSPACE}"
CMD [ "bash" ]


# ///////////////////////////////////////////////////////////////////////////////////////// norlab-ros2/dependencies ///



