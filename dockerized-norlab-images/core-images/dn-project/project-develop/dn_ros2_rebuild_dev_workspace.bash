#!/bin/bash -i
# =================================================================================================
# This script is used for setting up the development environment and building the project using
# colcon build system for ROS (Robot Operating System).
#
# Usage:
#         $ source dn_ros2_rebuild_dev_workspace.bash ["virtualization|native"]
#
# Arguments:
#         PARAM_ARCH:  (optional) The architecture type, could be 'native' or 'virtualization'.
#                      Default to 'native'
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

# (Priority) ToDo: implement an help menu
# (Priority) ToDo: refactor optional argument virtualization|native as flags
# (NICE TO HAVE) ToDo: consider merging script with DNA dn_project_core.build.aarch_aware_build_ros.bash
# (Priority) ToDo: move script to container-tools directory

# ....Set argument.................................................................................
PARAM_ARCH=${1:-"native"}

# ....Check pre-conditions.......................................................................
{
  test -n "${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'}" && \
  test -n "${ROS_DISTRO:?'Env variable need to be set and non-empty.'}";
} || n2st::print_msg_error_and_exit "Failed pre-condition checks!"

# ====Begin========================================================================================
n2st::print_formated_script_header "dn_ros2_rebuild_dev_workspace.bash" "${MSG_LINE_CHAR_INSTALLER}"
cd "${DN_DEV_WORKSPACE}" || exit 1

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Install dependencies
n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}apt-get update${MSG_END_FORMAT}...\n"
sudo apt-get update
echo
n2st::draw_horizontal_line_across_the_terminal_window "."

ROSDEP_UPDATE_FLAGS=("--rosdistro" "${ROS_DISTRO}" "--include-eol-distros")
n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}rosdep update ${ROSDEP_UPDATE_FLAGS[*]}${MSG_END_FORMAT}...\n"
rosdep update "${ROSDEP_UPDATE_FLAGS[@]}" || n2st::print_msg_error_and_exit "Failed rosdep update!"
echo
n2st::draw_horizontal_line_across_the_terminal_window "."


n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}rosdep fix-permissions${MSG_END_FORMAT}...\n"
rosdep fix-permissions
echo
n2st::draw_horizontal_line_across_the_terminal_window "."

# Note on rosdep install: looks at all the packages in the src directory and tries to find
# and install their dependencies on your platform.
ROSDEP_FLAGS=("--ignore-packages-from-source")
ROSDEP_FLAGS+=("--from-paths" "./src")
ROSDEP_FLAGS+=("--rosdistro" "${ROS_DISTRO}")
ROSDEP_FLAGS+=("--default-yes")
n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}rosdep install ${ROSDEP_FLAGS[*]}${MSG_END_FORMAT}...\n"
rosdep install "${ROSDEP_FLAGS[@]}" || n2st::print_msg_error_and_exit "Failed rosdep install!"
echo
n2st::draw_horizontal_line_across_the_terminal_window "."

n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}colcon version-check${MSG_END_FORMAT}...\n"
colcon version-check
echo
n2st::draw_horizontal_line_across_the_terminal_window "."

## colcon build step: rebuild everything in the catkin workspace DN_DEV_WORKSPACE
# shellcheck disable=SC1090

COLCON_FLAGS=()
if [[ "${PARAM_ARCH}" == "native" ]]; then
  n2st::print_msg "Builder is running on native architecture"
  COLCON_FLAGS+=("--symlink-install")
elif [[ "${PARAM_ARCH}" == "virtualization" ]]; then
  n2st::print_msg "Builder is running in architecture virtualisation"
  COLCON_FLAGS+=("--executor" "sequential")
fi

COLCON_FLAGS+=(
  "--cmake-clean-cache"
  "--cmake-args" "-DCMAKE_BUILD_TYPE=Release"
  "--event-handlers" "console_direct+"
)
n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}colcon build ${COLCON_FLAGS[*]}${MSG_END_FORMAT}...\n"
colcon build "${COLCON_FLAGS[@]}" || n2st::print_msg_error_and_exit "Failed colcon build!"
n2st::draw_horizontal_line_across_the_terminal_window "."

# ====Teardown=====================================================================================
n2st::print_formated_script_footer "dn_ros2_rebuild_dev_workspace.bash" "${MSG_LINE_CHAR_INSTALLER}"
popd >/dev/null || exit 1
