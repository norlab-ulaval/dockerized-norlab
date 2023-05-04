
# /// norlab-ros2/dependencies /////////////////////////////////////////////////////////////////////////////////////////
# This container is the base image for all other norlab-ros images: dev, deploy, ...
#
# References:
# - NVIDIA Container Runtime on Jetson: https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson
# - dusty-nv/jetson-containers: https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.noetic
# - ROS noetic installation from source: http://wiki.ros.org/noetic/Installation/Source


# Base image: nvidia linux 4 tegra (L4T) nvidia docker container official image
#   l4t-base: https://ngc.nvidia.com/catalog/containers/nvidia:l4t-base
#   l4t-ros-noetic-pytorch: https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic

# OS: Ubuntu[18.04 (melodic)|22.04.2 LTS (Jammy Jellyfish)], l4t

# JETPACK & L4T release version: https://developer.nvidia.com/embedded/jetpack-archive and https://www.stereolabs.com/blog/nvidia-jetson-l4t-and-jetpack-support/
# Jetpack 4.6 › tag: r32.6.1
# Jetpack 5.1 › tag: r35.2.1

#ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.6.1
ARG BASE_IMAGE=dustynv/ros:humble-pytorch-l4t-r35.2.1
FROM ${BASE_IMAGE} AS ros-base-image
LABEL org.opencontainers.image.authors="luc.coupal.1@ulaval.ca"

# ARCH: arm64-l4t, arm64-darwin, x86
ARG DN_IMAGE_ARCHITECTURE=NotSet
ENV DN_IMAGE_ARCHITECTURE=${DN_IMAGE_ARCHITECTURE}

# ROS pkg: ros_core, ros_base, desktop_full
# Note: base_image `dustynv/ros:humble-pytorch-l4t-r35.2.1` is ros_base as per https://hub.docker.com/layers/dustynv/ros/humble-pytorch-l4t-r35.2.1/images/sha256-85b6e35baf947074b19efbdd628e3d0d5e32d2563557b493721d9de535463316?context=explore
ARG DN_ROS_PKG=ros_base
ENV DN_ROS_PKG=${DN_ROS_PKG}

## Declared in D-NL › Deploy
## NorLab robot: warthog, warthog-xavier, xavier-nx-redleader, marmote ...
## DS_HOST_TYPE: Local, XavierWarthog,

ENV DN_DEV_WORKSPACE=$HOME/ros2_ws

# skip GUI dialog by setting everything to default
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR "/"

# ....install development utilities................................................................................
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
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# ....install ROS2 specific utilities..............................................................................
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        ros-dev-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p "${DN_DEV_WORKSPACE}/src"

# ....install sources..............................................................................................

## . . ros2_control Demos. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
## https://github.com/ros-controls/ros2_control_demos
#
#WORKDIR ${DN_DEV_WORKSPACE}/src
#RUN git clone https://github.com/ros-controls/ros2_control
#RUN git clone https://github.com/ros-controls/ros2_controllers
#RUN git clone https://github.com/ros-controls/ros2_control_demos
#
#WORKDIR ${DN_DEV_WORKSPACE}
#RUN apt-get update \
#    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash" \
#    && rosdep install --from-paths src --ignore-src -r -y \
#    && rm -rf /var/lib/apt/lists/*


## . . Deep Learning Nodes for ROS/ROS2. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
## https://github.com/dusty-nv/ros_deep_learning

#WORKDIR "~"
#RUN git clone --recursive https://github.com/dusty-nv/jetson-inference \
#    && cd jetson-inference \
#    && mkdir build \
#    && cd build \
#    && cmake ../ \
#    && make -j$(nproc) \
#    && sudo make install \
#    && sudo ldconfig

#RUN apt-get update \
#    && apt-get install --assume-yes --no-install-recommends \
#        ros-humble-vision-msgs \
#        ros-humble-launch-xml \
#        ros-humble-launch-yaml \
#    && rm -rf /var/lib/apt/lists/*

#WORKDIR ${DN_DEV_WORKSPACE}/src
#RUN git clone https://github.com/dusty-nv/ros_deep_learning
#
#WORKDIR ${DN_DEV_WORKSPACE}
#RUN apt-get update \
#    && /bin/bash -c "sudo -H pip install --upgrade jetson-stats" \
#    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash" \
#    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --default-yes  \
##    --os=ubuntu:jammy \
#    && rm -rf /var/lib/apt/lists/*

## . . ros2_jetson_stats. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#WORKDIR ${DN_DEV_WORKSPACE}/src
#RUN git clone --branch humble https://github.com/NVIDIA-AI-IOT/ros2_jetson_stats.git
#
#WORKDIR ${DN_DEV_WORKSPACE}
#RUN apt-get update \
#    && /bin/bash -c "sudo -H pip install --upgrade jetson-stats" \
#    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash" \
#    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --default-yes --os=ubuntu:jammy \
#    && rm -rf /var/lib/apt/lists/*

## . . icp_odom_with_twist. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#WORKDIR ${DN_DEV_WORKSPACE}/src
#RUN git clone https://github.com/norlab-ulaval/icp_odom_with_twist.git
#
#WORKDIR "${DN_DEV_WORKSPACE}"
#RUN apt-get update \
#    && rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} -y

## . . ros2/examples. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#WORKDIR ${DN_DEV_WORKSPACE}/src
#RUN git clone --branch humble https://github.com/ros2/examples
#
#WORKDIR ${DN_DEV_WORKSPACE}
#RUN apt-get update \
#    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash" \
#    && rosdep install --from-paths ./src --ignore-src -r --rosdistro ${ROS_DISTRO} --default-yes \
#    && rm -rf /var/lib/apt/lists/*

#WORKDIR ${DN_DEV_WORKSPACE}/src
#RUN git clone --branch humble https://github.com/ros2/examples \
#    && cd ${DN_DEV_WORKSPACE} \
#    && apt-get update \
#    && rosdep install --from-paths src --ignore-src -r --rosdistro ${ROS_DISTRO} --default-yes \
##    && rosdep install --from-paths ./src --ignore-src --rosdistro ${ROS_DISTRO} --default-yes --os=ubuntu:jammy \
##    && rosdep install --from-paths src --ignore-src -r --rosdistro ${ROS_DISTRO} --default-yes \
#    && rm -rf /var/lib/apt/lists/*

# (!) Quick-hack: `rosdep install` flag `-r` = Continue installing despite errors (ref https://github.com/IntelRealSense/realsense-ros/issues/2309)
# (NICE TO HAVE) ToDo: fixme!! (ref task NMO-163 ﹅→ rosdep install error for ros2/example repo)
# lsb_release -a # Check Ubuntu version

WORKDIR ${DN_DEV_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash \
        && colcon build --symlink-install"
#        && colcon build --symlink-install --merge-install"


#RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash \
#    && source ${DN_DEV_WORKSPACE}/install/setup.bash" \
#    && echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash" >> ~/.bashrc \
#    && echo "source ${DN_DEV_WORKSPACE}/install/setup.bash" >> ~/.bashrc

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash" \
    && echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash" >> ~/.bashrc


## ===INstall Python 3 step DONE========================================================================================
# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
WORKDIR "${DN_DEV_WORKSPACE}"
CMD [ "bash" ]


# ///////////////////////////////////////////////////////////////////////////////////////// norlab-ros2/dependencies ///



