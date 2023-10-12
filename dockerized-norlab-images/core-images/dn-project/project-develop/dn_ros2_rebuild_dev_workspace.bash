#!/bin/bash -i

#set -e # exit script if any statement returns a non-true return value
#set -v

cd "${DN_DEV_WORKSPACE}"

# Install dependencies
sudo apt-get update
# rosdep install: looks at all the packages in the src directory and tries to find and install their dependencies on your platform
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro "${ROS_DISTRO}" --default-yes

## colcon build step: rebuild everything in the catkin workspace DN_DEV_WORKSPACE
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --symlink-install
# --merge-install
source "${DN_DEV_WORKSPACE}/install/setup.bash"

cd "${DN_PROJECT_PATH}"

