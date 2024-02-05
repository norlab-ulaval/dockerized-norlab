#!/bin/bash

# ===============================================================================================
# Pre docker command execution callback
#
# Usage:
#   $ callback_execute_compose_pre()
#
# Globals:
#   Read COMPOSE_FILE
#   Read DEPENDENCIES_BASE_IMAGE
#   Read DEPENDENCIES_BASE_IMAGE_TAG
#
# =================================================================================================
function dn::callback_execute_compose_pre() {

  # ....Reformat nvcr.io base image tag............................................................
  if [[ ${BASE_IMAGE} == "nvcr.io/nvidia/pytorch" ]]; then
    if [[ ${TAG_OS_VERSION} == jammy ]]; then
      CONVERTED_TAG_OS_VERSION=23
    elif [[ ${TAG_OS_VERSION} == focal ]]; then
      CONVERTED_TAG_OS_VERSION=22
    else
      n2st::print_msg_error_and_exit "TAG_OS_VERSION=${TAG_OS_VERSION} not suported yet by base image callback"
    fi
    export DEPENDENCIES_BASE_IMAGE_TAG="${CONVERTED_TAG_OS_VERSION}.${BASE_IMG_TAG_PREFIX}"
  fi

  # ....Export image tag for squashed base image use...............................................
  export DN_IMAGE_TAG_NO_ROS="DN-${REPOSITORY_VERSION}-${DN_IMAGE_TAG_END}"


  # ....Fetch base image environment variables.....................................................
  if [[ ! -d ${NBS_COMPOSE_DIR:?err} ]]; then
    n2st::print_msg_error_and_exit "The directory ${NBS_COMPOSE_DIR} is unreachable"
  fi

  if [[ $(basename "${COMPOSE_FILE}") == "docker-compose.squash.build.yaml" ]]; then

    # ex: dustynv/pytorch:2.1-r35.2.1
    DOCKER_IMG="${DEPENDENCIES_BASE_IMAGE:?err}:${DEPENDENCIES_BASE_IMAGE_TAG:?err}"
    n2st::print_msg "Pulling DOCKER_IMG=${DOCKER_IMG}"

    # shellcheck disable=SC2046
    docker pull "${DOCKER_IMG}" \
      && export $(docker inspect --format='{{range .Config.Env}}{{println .}}{{end}}' "${DOCKER_IMG}" \
        | grep \
          -e CUDA_HOME= \
          -e NVIDIA_ \
          -e LD_LIBRARY_PATH= \
          -e PATH= \
          -e ROS_ \
          -e RMW_IMPLEMENTATION= \
          -e LD_PRELOAD= \
          -e OPENBLAS_CORETYPE= \
          -e PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION= \
          -e TORCH_HOME= \
        | sed 's;^CUDA_HOME;BASE_IMG_ENV_CUDA_HOME;' \
        | sed 's;^NVIDIA_;BASE_IMG_ENV_NVIDIA_;' \
        | sed 's;^PATH;BASE_IMG_ENV_PATH;' \
        | sed 's;^LD_LIBRARY_PATH;BASE_IMG_ENV_LD_LIBRARY_PATH;' \
        | sed 's;^ROS_;BASE_IMG_ENV_ROS_;' \
        | sed 's;^RMW_IMPLEMENTATION;BASE_IMG_ENV_RMW_IMPLEMENTATION;' \
        | sed 's;^LD_PRELOAD;BASE_IMG_ENV_LD_PRELOAD;' \
        | sed 's;^OPENBLAS_CORETYPE;BASE_IMG_ENV_OPENBLAS_CORETYPE;' \
        | sed 's;^PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION;BASE_IMG_ENV_PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION;' \
        | sed 's;^TORCH_HOME;BASE_IMG_ENV_TORCH_HOME;' \
       )

    n2st::print_msg "Passing the following environment variable from ${MSG_DIMMED_FORMAT}${DEPENDENCIES_BASE_IMAGE}:${DEPENDENCIES_BASE_IMAGE_TAG}${MSG_END_FORMAT} to ${MSG_DIMMED_FORMAT}${DN_HUB:?err}/dockerized-norlab-base-image-squashed:${DN_IMAGE_TAG:?err}${MSG_END_FORMAT}:
      ${MSG_DIMMED_FORMAT}\n$(printenv | grep -e BASE_IMG_ENV_ | sed 's;BASE_IMG_ENV_;    ;')
      ${MSG_END_FORMAT}"
  else
    n2st::print_msg "Skiping base image environment variable fetching"
  fi

}

