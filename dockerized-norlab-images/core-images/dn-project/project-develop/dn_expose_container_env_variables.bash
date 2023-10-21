#!/bin/bash
#
# Fetch container environment variables and expose them to the host
# through a mounted volume "dockerized-norlab-tools/dn_container_env_variable/"
# making them availbale to sourcing in any IDEs.
# e.g.: PyCharm run configuration using EnvFile plugin (https://github.com/Ashald/EnvFile)
#
#
#
# Usage:
#   $ bash dn_expose_container_env_variables.bash
#
# Globals:
#   Read ros related env variables:
#       ROS_DISTRO, ROS_ROOT, ROS_VERSION, ROS_PYTHON_VERSION, ROS_DOMAIN_ID,
#       ROS_LOCALHOST_ONLY, AMENT_PREFIX_PATH, CMAKE_PREFIX_PATH,
#       COLCON_PREFIX_PATH, PKG_CONFIG_PATH, PYTHONPATH, PATH,
#       LD_LIBRARY_PATH, HOSTNAME, RMW_IMPLEMENTATION, OPENBLAS_CORETYPE
#   Read display and x11 forwarding related env variables:
#       DISPLAY, LIBGL_ALWAYS_INDIRECT, QT_X11_NO_MITSHM,
#       NVIDIA_VISIBLE_DEVICES, NVIDIA_DRIVER_CAPABILITIES
#   Read Dockerized-NorLab related env variables:
#
# Outputs:
#   to stdout
#

touch "/dn_container_env_variable/.env.dn_expose_${DN_CONTAINER_NAME:?'Variable unset'}"

( \
  echo ""; \
  echo "# ROS related env"; \
  echo "ROS_DISTRO=${ROS_DISTRO}"; \
  echo "ROS_ROOT=${ROS_ROOT}"; \
  echo "ROS_VERSION=${ROS_VERSION}"; \
  echo "ROS_PYTHON_VERSION=${ROS_PYTHON_VERSION}"; \
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"; \
  echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"; \
  echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH}"; \
  echo "CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}"; \
  echo "COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH}"; \
  echo "PKG_CONFIG_PATH=${PKG_CONFIG_PATH}"; \
  echo "PYTHONPATH=\${PYTHONPATH}:${PYTHONPATH}"; \
  echo "PATH=\${PATH}:${PATH}"; \
  echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"; \
  echo "HOSTNAME=${HOSTNAME}"; \
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"; \
  echo "OPENBLAS_CORETYPE=${OPENBLAS_CORETYPE}"; \
  echo ""; \
  echo "# Cuda related env"; \
  echo "CUDA_HOME=${CUDA_HOME}"; \
  echo "NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES}"; \
  echo "NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES}"; \
  echo "LD_PRELOAD=${LD_PRELOAD}"; \
  echo ""; \
  echo "# Display forwarding related env"; \
  echo "DISPLAY=${DISPLAY}"; \
  echo "LIBGL_ALWAYS_INDIRECT=${LIBGL_ALWAYS_INDIRECT}"; \
  echo "QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM}"; \
  echo ""; \
  echo "# Dockerized-NorLab related env"; \
  echo "DN_DEV_WORKSPACE=${DN_DEV_WORKSPACE}"; \
  echo "DN_PROJECT_PATH=${DN_PROJECT_PATH}"; \
  echo "DN_CONTAINER_NAME=${DN_CONTAINER_NAME}"; \
  echo "DN_ACTIVATE_POWERLINE_PROMT=${DN_ACTIVATE_POWERLINE_PROMT}"; \
  echo "DN_GDB_SERVER_PORT=${DN_GDB_SERVER_PORT}"; \
  echo "DN_PROJECT_GIT_NAME=${DN_PROJECT_GIT_NAME}"; \
  echo "DN_SSH_SERVER_PORT=${DN_SSH_SERVER_PORT}"; \
  echo "DN_SSH_SERVER_USER=${DN_SSH_SERVER_USER}"; \
  echo "DN_PROJECT_GIT_DOMAIN=${DN_PROJECT_GIT_DOMAIN}"; \
  echo "PYTHONUNBUFFERED=${PYTHONUNBUFFERED}"; \
  echo ""; \
) > "/dn_container_env_variable/.env.dn_expose_${DN_CONTAINER_NAME}"


