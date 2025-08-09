#!/bin/bash
# =================================================================================================
# Dockerized-NorLab project-core image initialization script
# Is executed by '.dockerized_norlab_project/configuration/Dockerfile' in a DN project image
#
# Usage:
#   source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_project_core_finalize.bash
#
# Globals:
#   Read DN_PROJECT_USER
#   Read DN_PROJECT_USER_HOME
#   Read DN_PROJECT_UID
#   Read DN_PROJECT_GID
#   Read DN_PROJECT_PATH
#   Read DN_DEV_WORKSPACE
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
set -e
pushd "$(pwd)" >/dev/null || exit 1

# (CRITICAL) ToDo: unit-test (ref task NMO-548 and RLRP-213)

function dn::finalize_dockerized_norlab_project() {

  # ....Check pre-conditions.......................................................................
  {
    test -n "${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_USER_HOME:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_UID:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_GID:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_PATH:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_GIT_NAME:?'Env variable need to be set and non-empty.'}" ;
  } || n2st::print_msg_error_and_exit "Failed pre-condition check!"

  # ....Configure python system path discovery for ros2............................................
  {
    n2st::print_msg "Add ROS2 lib to python sys path..."
    source "/dockerized-norlab/dockerized-norlab-images/container-tools/dn_add_ros2_pythonpath.bash" "pth" || exit 1
  }

  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "\033[1;31m[DN error]\033[0m This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DN error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }
  dn::finalize_dockerized_norlab_project || n2st::print_msg_error_and_exit "dn::finalize_dockerized_norlab_project exited with error!"
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
