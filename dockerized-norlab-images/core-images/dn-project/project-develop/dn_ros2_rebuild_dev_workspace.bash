#!/bin/bash -i
# =================================================================================================
# This script is used for setting up the development environment and building the project using
# colcon build system for ROS (Robot Operating System).
#
# Usage:
#         $ source dn_ros2_rebuild_dev_workspace.bash "[virtualization|native]"
#
# Arguments:
#         PARAM_ARCH:  The architecture type, could be 'native' or 'virtualization'.
#
# Globals:
#   Read:
#         DN_DEV_WORKSPACE: Path to the development workspace.
#         ROS_DISTRO: Name of the ROS distribution to use.
#
# Outputs:
#         Writes the build status, error messages and logs to stdout.
#
# =================================================================================================
set -e
pushd "$(pwd)" >/dev/null || exit 1

# ....Set argument.................................................................................
PARAM_ARCH=$1

# ....Check pre-conditions.......................................................................
{
  test -n "${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'}" && \
  test -n "${ROS_DISTRO:?'Env variable need to be set and non-empty.'}" ;
} || exit 1

# ====Begin========================================================================================
cd "${DN_DEV_WORKSPACE}" || exit 1

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${DN_DEV_WORKSPACE}/install/setup.bash"

# Install dependencies
sudo apt-get update --fix-missing
rosdep update --rosdistro "${ROS_DISTRO}" --include-eol-distros
rosdep fix-permissions

# Note on rosdep install: looks at all the packages in the src directory and tries to find
# and install their dependencies on your platform.
rosdep install \
  --ignore-packages-from-source \
  --from-paths ./src \
  --rosdistro "${ROS_DISTRO}" \
  --default-yes

colcon version-check

## colcon build step: rebuild everything in the catkin workspace DN_DEV_WORKSPACE
# shellcheck disable=SC1090

COLCON_FLAGS=()
if [[ "${PARAM_ARCH}" == "native" ]]; then
  echo -e "Builder is running on native architecture"
  COLCON_FLAGS+=("--symlink-install")
elif [[ "${PARAM_ARCH}" == "virtualization" ]]; then
  echo -e "Builder is running in architecture virtualisation"
  COLCON_FLAGS+=("--executor" "sequential")
fi

COLCON_FLAGS+=(
  "--cmake-clean-cache"
  "--cmake-args" "-DCMAKE_BUILD_TYPE=Release"
  "--event-handlers" "console_direct+"
)
echo -e "COLCON_FLAGS=( ${COLCON_FLAGS[*]} )"
colcon build "${COLCON_FLAGS[@]}"



# ====Teardown=====================================================================================
popd >/dev/null || exit 1

