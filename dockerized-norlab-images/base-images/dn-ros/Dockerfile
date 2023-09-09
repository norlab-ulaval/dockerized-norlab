
ARG BASE_IMAGE=dustynv/ros
ARG BASE_IMAGE_TAG=foxy-pytorch-l4t-r35.2.1
FROM ${BASE_IMAGE}:${BASE_IMAGE_TAG}

ARG IS_TEAMCITY_RUN
ENV IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:-false}

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

ENV TZ=Etc/UTC
ENV TERM=${TERM:-"xterm-256color"}

# ====Begin install============================================================================================

ENV DN_DEV_WORKSPACE=$HOME/ros2_ws

# ....install ROS2 specific utilities.........................................................................
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        ros-dev-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir  \
     osrf-pycommon

RUN mkdir -p "${DN_DEV_WORKSPACE}/src"

# ....install sources.........................................................................................

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


WORKDIR ${DN_DEV_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash \
        && colcon build --symlink-install"
#        && colcon build --symlink-install --merge-install"

# (CRITICAL) ToDo: add this step to pycharm-debugger
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash" \
    && echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash" >> ~/.bashrc


WORKDIR "${DN_DEV_WORKSPACE}"
CMD [ "bash" ]



