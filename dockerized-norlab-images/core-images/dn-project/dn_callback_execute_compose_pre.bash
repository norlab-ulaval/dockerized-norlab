#!/bin/bash

# ===============================================================================================
# Pre docker command execution callback
#
# Usage:
#   $ source dn_callback_execute_compose_pre.bash && callback_execute_compose_pre
#
# Notes:
#   See utilities/tmp/README.md for details on role of `dockerized-norlab-project-mock`
#
# Globals:
#   Read DN_PATH
#
# =================================================================================================
function dn::callback_execute_compose_pre() {
  n2st::print_formated_script_header "dn_callback_execute_compose_pre.bash" "${MSG_LINE_CHAR_UTIL}"

  test -n "${DN_PATH:?err}" || { n2st::print_msg_error "Env variable DN_PATH need to be set and non-empty." && return 1 ;}
  test -d "${DN_PATH}/utilities/tmp" || { n2st::print_msg_error "The directory ${DN_PATH}/utilities/tmp is unreachable" && return 1 ;}

  if [[ -d "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock/.git" ]]; then
      # Delete git cloned repo
      rm -rf "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock"
  fi
  if [[ ! -d "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock" ]]; then
    mkdir "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock"
  fi
  git clone https://github.com/norlab-ulaval/dockerized-norlab-project-mock.git \
    "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock" \
    || { n2st::print_msg_error "Could not clone dockerized-norlab-project-mock" && return 1 ;}

  # ....Sanity check...............................................................................
  test -d "${DN_PATH}/utilities/tmp" || { n2st::print_msg_error "The directory ${DN_PATH}/utilities/tmp is unreachable" && return 1 ;}
  test -d "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock/.git" \
  || { \
    tree -a -L 2 "${DN_PATH}/utilities/tmp" &&
    n2st::print_msg_error "The directory ${DN_PATH}/utilities/tmp is unreachable" && return 1 ;
    }

  n2st::print_formated_script_footer "dn_callback_execute_compose_pre.bash" "${MSG_LINE_CHAR_UTIL}"
}
