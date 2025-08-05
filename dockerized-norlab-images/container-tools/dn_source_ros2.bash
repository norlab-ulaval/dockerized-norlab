#!/bin/bash
# =================================================================================================
# ROS2 environment sourcing functions. Add functions for sourcing ros2 overlay, underlay or both.
#
# Usage:
#   $ bash dn_source_ros2.bash
#   $ dn::source_ros2_underlay_only ["The caller name"]
#   $ dn::source_ros2_overlay_only ["overlay_script_name.bash" ["The caller name"]]
#   $ dn::source_ros2 ["overlay_script_name.bash"]
#
# Note
#  - The script functions is meant to be portable i.e., no dependency toward N2ST lib
#  - The script functions are exported in DN dependencies images by
#    import_dockerized_norlab_container_tools.bash, so they should be available in RUN bloc whitout
#    having to source dn_source_ros2.bash.
#
# Global:
#   read ROS_DISTRO
#   read DN_DEV_WORKSPACE
#
# =================================================================================================


#DN_SHOW_DEBUG_INFO=false # (CRITICAL) ToDo: on task end >> switch to false ←
MSG_DIMMED_FORMAT="\033[1;2m"
MSG_END_FORMAT="\033[0m"

function _show_debug_info() {
  local caller="$1"
  echo -e "\033[1;33m[DN trace]\033[0m Info from $1
  ${MSG_DIMMED_FORMAT}
  BASH_SOURCE: ${BASH_SOURCE[*]}
  realpath: $(realpath .)
  \$0: $0
  ${MSG_END_FORMAT}"
}

if [[ "${DN_SHOW_DEBUG_INFO}" == true ]] && [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  _show_debug_info "script dn_source_ros2.bash"
fi

# Source ROS2 environment (the underlay)
function dn::source_ros2_underlay_only() {
  local caller="${1:-"${BASH_SOURCE[1]}"}"
  if [[ "${DN_SHOW_DEBUG_INFO}" == true ]] && [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
    _show_debug_info "function dn::source_ros2_underlay_only()"
  fi
  if [[ -n "${ROS_DISTRO:?'Environment variable is not set!'}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash"  ]]; then
    echo -e "${MSG_DIMMED_FORMAT}sourcing underlay /opt/ros/${ROS_DISTRO}/setup.bash from ${caller}${MSG_END_FORMAT}"
    source "/opt/ros/${ROS_DISTRO}/setup.bash" || return 1
  else
    echo -e "\033[1;31m[DN error]\033[0m /opt/ros/${ROS_DISTRO}/setup.bash is unreachable in ${caller}!" 1>&2
    return 1
  fi
  return 0
}

# Source ROS2 workspace environment (the overlay)
function dn::source_ros2_overlay_only() {
  local overlay_script="${1:-"local_setup.bash"}"
  #local overlay_script=setup.bash
  local caller="${2:-"${BASH_SOURCE[1]}"}"
  if [[ "${DN_SHOW_DEBUG_INFO}" == true ]] && [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
    _show_debug_info "function dn::source_ros2_overlay_only()"
    echo -e "${MSG_DIMMED_FORMAT}  overlay_script: ${overlay_script}\n  caller: ${caller}\n  DN_DEV_WORKSPACE: ${DN_DEV_WORKSPACE}${MSG_END_FORMAT}\n"
  fi
  if [[ -f "${DN_DEV_WORKSPACE:?'Environment variable is not set!'}/install/${overlay_script}" ]]; then
    echo -e "${MSG_DIMMED_FORMAT}sourcing overlay ${DN_DEV_WORKSPACE}/install/${overlay_script} from ${caller}${MSG_END_FORMAT}"
    source "${DN_DEV_WORKSPACE}/install/${overlay_script}" || return 1
  else
    echo -e "\033[1;33m[DN warning]\033[0m ${DN_DEV_WORKSPACE}/install/local_setup.bash is unreachable in ${caller}! Skip sourcing overlay."
  fi
  return 0
}

# Source ROS2 workspace environment (both the underlay and the overlay)
function dn::source_ros2() {
  local overlay_script="${1:-"local_setup.bash"}"
  local caller="${BASH_SOURCE[1]}"
  if [[ "${DN_SHOW_DEBUG_INFO}" == true ]] && [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
    _show_debug_info "function dn::source_ros2()"
  fi
  echo
  if [[ -n "${ROS_DISTRO}"  ]]; then
    dn::source_ros2_underlay_only "${caller}" || return 1
    if [[ -f "${DN_DEV_WORKSPACE:?'Environment variable is not set!'}/install/${overlay_script}" ]]; then
      dn::source_ros2_overlay_only "local_setup.bash" "${caller}" || return 1
    fi
  else
    echo -e "\033[1;33m[DN warning]\033[0m ROS_DISTRO is missing! Skipping dn::source_ros2"
  fi
  echo
  return 0
}



