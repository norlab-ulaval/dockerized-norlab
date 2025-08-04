#!/bin/bash
# =================================================================================================
# ROS2 environment sourcing functions. Add functions for sourcing ros2 overlay, underlay or both.
#
# Usage:
#   $ bash dn_source_ros2.bash
#   $ dn::source_ros2_underlay_only
#   $ dn::source_ros2_overlay_only
#   $ dn::source_ros2
#
# Note
#   The script functions are exported in DN dependencies images by
#   import_dockerized_norlab_container_tools.bash, so they should be available in RUN bloc whitout
#   having to source dn_source_ros2.bash.
#
# Global:
#   read ROS_DISTRO
#
# =================================================================================================

_dn_error_prefix="\033[1;31m[DN error]\033[0m"
_debug_log=false

function _show_debug_info() {
  local caller="$1"
  echo -e "\nfrom $1
  BASH_SOURCE: ${BASH_SOURCE[*]}
  realpath: $(realpath .)
  \$0: $0
  "
}

if [[ "${_debug_log}" == true ]]; then
  _show_debug_info "script dn_source_ros2.bash"
fi

# Source ROS2 environment (the underlay)
function dn::source_ros2_underlay_only() {
  local caller="${1:-"${BASH_SOURCE[1]}"}"
  if [[ "${_debug_log}" == true ]]; then
    _show_debug_info "function dn::source_ros2_underlay_only()"
  fi
  if [[ -n "${ROS_DISTRO:?'Environment variable is not set!'}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash"  ]]; then
    echo -e "\033[1;2msourcing underlay /opt/ros/${ROS_DISTRO}/setup.bash from ${caller}\033[0m"
    source "/opt/ros/${ROS_DISTRO}/setup.bash" || return 1
  else
    echo -e "${_dn_error_prefix} /opt/ros/${ROS_DISTRO}/setup.bash is unreachable in ${caller}!" 1>&2
    return 1
  fi
  return 0
}

# Source ROS2 workspace environment (the overlay)
function dn::source_ros2_overlay_only() {
  local caller="${1:-"${BASH_SOURCE[1]}"}"
  local overlay_script=local_setup.bash
  #local overlay_script=setup.bash
  if [[ "${_debug_log}" == true ]]; then
    _show_debug_info "function dn::source_ros2_overlay_only()"
  fi
  if [[ -f "${DN_DEV_WORKSPACE:?'Environment variable is not set!'}/install/${overlay_script}" ]]; then
    echo -e "\033[1;2msourcing overlay ${DN_DEV_WORKSPACE}/install/${overlay_script} from ${caller}\033[0m"
    source "${DN_DEV_WORKSPACE}/install/${overlay_script}" || return 1
  else
    echo -e "${_dn_error_prefix} ${DN_DEV_WORKSPACE}/install/local_setup.bash is unreachable in ${caller}!" 1>&2
    return 1
  fi
  return 0
}

# Source ROS2 workspace environment (both the underlay and the overlay)
function dn::source_ros2() {
  local caller="${BASH_SOURCE[1]}"
  if [[ "${_debug_log}" == true ]]; then
    _show_debug_info "function dn::source_ros2()"
  fi
  echo
  dn::source_ros2_underlay_only "${caller}" || return 1
  dn::source_ros2_overlay_only "${caller}" || return 1
  echo
  return 0
}



