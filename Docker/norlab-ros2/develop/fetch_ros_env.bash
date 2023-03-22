#!/bin/bash

# Credit: Anas Abou Allaban
#   https://www.allaban.me/posts/2020/08/ros2-setup-ide-docker/

ROS_ENV="AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PKG_CONFIG_PATH PYTHONPATH
LD_LIBRARY_PATH PATH ROS_DISTRO ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_VERSION
ROS_ETC_DIR
ROS_ROOT
ROS_MASTER_URI
ROS_PACKAGE_PATH
ROSLISP_PACKAGE_DIRECTORIES
DS_PYCHARM_DEV_SERVER_PORT
DS_PYCHARM_DEV_USER
DS_DEV_WORKSPACE
DS_TARGET_PROJECT_SRC_REPO
DS_HOST_TYPE
DS_PYTHON3_VERSION
DS_ROS_PKG
DS_IMAGE_ARCHITECTURE
DS_DEV_WORKSPACE
MASTER_HOSTNAME
HOSTNAME
ROSLAUNCH_SSH_UNKNOWN
MASTER_USER
"
ENV_STRING=""
for e in ${ROS_ENV}; do
  ENV_STRING+="$e=${!e};"
done

echo $ENV_STRING
