#!/bin/bash
#
# Fetch container ROS specific environment variables, seperated by ';' ready to copy/paste
#
# Usage:
#   $ bash dn_expose_container_env_variables.bash
#   ROS_DISTRO=foxy;ROS_ROOT=/opt/ros/foxy;ROS_VERSION=2;ROS_PYTHON_VERSION=3;ROS_DOMAIN_ID=1;ROS_LOCALHOST_ONLY=0;AMENT_PREFIX_PATH=/opt/ros/foxy:/opt/ros/foxy/install;CMAKE_PREFIX_PATH=/opt/ros/foxy/install;COLCON_PREFIX_PATH=/opt/ros/foxy/install;PKG_CONFIG_PATH=/opt/ros/foxy/install/lib/aarch64-linux-gnu/pkgconfig:/opt/ros/foxy/install/lib/pkgconfig;PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:/opt/ros/foxy/install/lib/python3.8/site-packages;PATH=/opt/ros/foxy/bin:/opt/ros/foxy/install/bin:/usr/local/cuda/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin;LD_LIBRARY_PATH=/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib:/opt/ros/foxy/install/opt/yaml_cpp_vendor/lib:/opt/ros/foxy/install/lib:/usr/local/cuda/lib64:/usr/local/cuda/lib64:;HOSTNAME=28530ae336af;
#
# Globals:
#   Read ROS_DISTRO, ROS_ROOT, ROS_VERSION, ROS_PYTHON_VERSION, ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, AMENT_PREFIX_PATH, CMAKE_PREFIX_PATH, COLCON_PREFIX_PATH, PKG_CONFIG_PATH, PYTHONPATH, PATH, LD_LIBRARY_PATH, HOSTNAME, RMW_IMPLEMENTATION
#
# Outputs:
#   to stdout
#
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

ENV_STRING=""
for e in ${ROS_ENV}; do
  ENV_STRING+="$e=${!e};"
done

echo "$ENV_STRING"

