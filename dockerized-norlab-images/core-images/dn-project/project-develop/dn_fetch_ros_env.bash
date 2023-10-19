#!/bin/bash

# Credit: Anas Abou Allaban
#   https://www.allaban.me/posts/2020/08/ros2-setup-ide-docker/

ROS_ENV="
ROS_DISTRO
ROS_ROOT
ROS_VERSION
ROS_PYTHON_VERSION
ROS_DOMAIN_ID
ROS_LOCALHOST_ONLY
AMENT_PREFIX_PATH
CMAKE_PREFIX_PATH
COLCON_PREFIX_PATH
PKG_CONFIG_PATH
PYTHONPATH
PATH
LD_LIBRARY_PATH
HOSTNAME
RMW_IMPLEMENTATION
"

DISPLAY_FORWARDING="
DISPLAY
LIBGL_ALWAYS_INDIRECT
QT_X11_NO_MITSHM
"

ENV_STRING=""
for e in ${ROS_ENV}; do
  ENV_STRING+="$e=${!e};"
done

for e in ${DISPLAY_FORWARDING}; do
  ENV_STRING+="$e=${!e};"
done

echo "$ENV_STRING"

printenv | grep -i -e DN_ -e DN_PROJECT
