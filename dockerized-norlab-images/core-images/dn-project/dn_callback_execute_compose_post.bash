#!/bin/bash

# ===============================================================================================
# Post docker command execution callback
#
# Notes:
#   - It is source and executed by dn_execute_compose.bash
#   - Its required to be at the same level as its corresponding docker-compose.*.yaml
#
# Usage:
#   $ source dn_callback_execute_compose_post.bash && dn::callback_execute_compose_post
#
# Notes:
#   See utilities/tmp/README.md for details on role of `dockerized-norlab-project-mock`
#
# Globals:
#   Read DN_PATH
#
# =================================================================================================
function dn::callback_execute_compose_post() {
  n2st::print_formated_script_header "dn_callback_execute_compose_post.bash" "${MSG_LINE_CHAR_UTIL}"

  test -n "${DN_PATH:?err}" || { n2st::print_msg_error "Env variable DN_PATH need to be set and non-empty." && return 1 ; }
  test -d "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock" \
    || { n2st::print_msg_error "The directory ${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock is unreachable" && return 1 ; }

  # Delete git cloned repo
  rm -rf "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock"

  # Setup placeholder
  mkdir "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock"

  # ....Sanity check...............................................................................
  test -d "${DN_PATH}/utilities" || { n2st::print_msg_error "The directory ${DN_PATH}/utilities is unreachable" && return 1 ; }
  test ! -d "${DN_PATH}/utilities/tmp/dockerized-norlab-project-mock/.git" \
  || { \
    tree -a -L 2 "${DN_PATH}/utilities" &&
    n2st::print_msg_error "The directory ${DN_PATH}/utilities is unreachable" && return 1 ;
    }

  n2st::print_formated_script_footer "dn_callback_execute_compose_post.bash" "${MSG_LINE_CHAR_UTIL}"
  return 0
}
