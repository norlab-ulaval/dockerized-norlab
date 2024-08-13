#!/bin/bash

# ===============================================================================================
# Pre docker command execution callback
#
# Usage:
#   $ source dn_callback_execute_compose_pre.bash && callback_execute_compose_pre
#
# Globals:
#   Read DN_PATH
#
# =================================================================================================
function dn::callback_execute_compose_pre() {
  set -e

  n2st::print_formated_script_header "dn_callback_execute_compose_pre.bash" "${MSG_LINE_CHAR_UTIL}"

  test -n "${DN_PATH:?err}" || n2st::print_msg_error_and_exit "Env variable DN_PATH need to be set and non-empty."
  test -d "${DN_PATH}/utilities/tmp" || n2st::print_msg_error_and_exit "The directory ${DN_PATH}/utilities/tmp is unreachable"

  if [[ -d "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock" ]]; then
      # Delete git cloned repo
      rm -rf "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock"
  fi
  if [[ ! -d "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock" ]]; then
    mkdir "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock"
  fi
  git clone https://github.com/norlab-ulaval/dockerized-norlab-project-mock.git \
    "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock" \
    || n2st::print_msg_error_and_exit "Could not clone dockerized-norlab-project-mock"

  # ....Saniti check...............................................................................
  test -d "${DN_PATH}/utilities/tmp" || n2st::print_msg_error_and_exit "The directory ${DN_PATH}/utilities/tmp is unreachable"
  test -d "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock/.git" \
  || { \
    tree -a -L 2 "${DN_PATH}/utilities/tmp" &&
    n2st::print_msg_error_and_exit "The directory ${DN_PATH}/utilities/tmp is unreachable" ;
    }

  n2st::print_formated_script_footer "dn_callback_execute_compose_pre.bash" "${MSG_LINE_CHAR_UTIL}"
}
