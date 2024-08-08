#!/bin/bash

function dn::callback_execute_compose_post() {
  n2st::print_formated_script_header "dn_callback_execute_compose_post.bash" "${MSG_LINE_CHAR_UTIL}"

  test -n "${DN_PATH:?err}" || n2st::print_msg_error_and_exit "Env variable DN_PATH need to be set and non-empty."
  test -d "${DN_PATH}/utilities/dockerized-norlab-project-mock" \
    || n2st::print_msg_error_and_exit "The directory ${DN_PATH}/utilities/dockerized-norlab-project-mock is unreachable"

  if [[ -d "${DN_PATH}/utilities/dockerized-norlab-project-mock"  ]]; then
    rm -rf "${DN_PATH}/utilities/dockerized-norlab-project-mock"
  fi

  # ....Saniti check...............................................................................
  tree -a -L 2 "${DN_PATH}/utilities"
  test -d "${DN_PATH}/utilities" || n2st::print_msg_error_and_exit "The directory ${DN_PATH}/utilities is unreachable"

  n2st::print_formated_script_footer "dn_callback_execute_compose_post.bash" "${MSG_LINE_CHAR_UTIL}"
}
