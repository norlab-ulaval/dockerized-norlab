#!/bin/bash -i

#set -e # exit script if any statement returns a non-true return value
#set -v

PARAM_ARCH=$1


cd "${DN_DEV_WORKSPACE}"

# Install dependencies
sudo apt-get update
# rosdep install: looks at all the packages in the src directory and tries to find and install their dependencies on your platform
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro "${ROS_DISTRO}" --default-yes

## colcon build step: rebuild everything in the catkin workspace DN_DEV_WORKSPACE
source "/opt/ros/${ROS_DISTRO}/setup.bash"

COLCON_FLAGS=()
if [[ "${PARAM_ARCH}" == "native" ]]; then
  echo -e "Builder is running on native architecture"
  COLCON_FLAGS+=(--symlink-install)
elif [[ "${PARAM_ARCH}" == "virtualization" ]]; then
  echo -e "Builder is running in architecture virtualisation"
  COLCON_FLAGS+=(--executor sequential)
fi
COLCON_FLAGS+=(
  --cmake-clean-cache
  --cmake-args -DCMAKE_BUILD_TYPE=Release
  --event-handlers console_direct+
)

echo -e "COLCON_FLAGS=( ${COLCON_FLAGS[*]} )"
colcon build "${COLCON_FLAGS[@]}"
source "${DN_DEV_WORKSPACE}/install/setup.bash"

cd "${DN_PROJECT_PATH}" || exit 1

