#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

ROS_ENV_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
echo "sourcing   ${ROS_ENV_SETUP}"
source "${ROS_ENV_SETUP}"

ROS_DEVEL_ENV_SETUP="${DS_DEV_WORKSPACE}/install/setup.bash"
echo "sourcing   ${ROS_DEVEL_ENV_SETUP}"
source "${ROS_DEVEL_ENV_SETUP}"

echo
echo "  Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment variable. "
echo "  It should include the directory you're in: /home/<youruser>/ros2_ws/src:/opt/ros/melodic/share"
echo
printenv | grep ROS
echo

exec "$@"
