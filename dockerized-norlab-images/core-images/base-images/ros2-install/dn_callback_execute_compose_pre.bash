#!/bin/bash


# ===============================================================================================
# Pre docker command execution callback
#
# Usage:
#   $ callback_execute_compose_pre()
#
# Globals:
#   Read REPOSITORY_VERSION
#   Read DN_IMAGE_TAG_END
#   Read NBS_COMPOSE_DIR
#
# =================================================================================================
function dn::callback_execute_compose_pre() {

  # ....Export image tag for squashed base image use...............................................
  export DN_IMAGE_TAG_NO_ROS="DN-${REPOSITORY_VERSION}-${DN_IMAGE_TAG_END}"

  # ....Fetch base image environment variables.....................................................
  if [[ ! -d ${NBS_COMPOSE_DIR:?err} ]]; then
    n2st::print_msg_error_and_exit "The directory ${NBS_COMPOSE_DIR} is unreachable"
  fi

  # ....Execute cuda squash base image logic.......................................................

  n2st::print_msg_warning "DN_IMAGE_TAG_NO_ROS=${DN_IMAGE_TAG_NO_ROS}"
}

