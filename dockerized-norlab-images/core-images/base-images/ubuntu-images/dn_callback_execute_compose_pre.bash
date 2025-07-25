#!/bin/bash

declare -x UBUNTU_VERSION_MAJOR

function dn::cuda_squash_image_logic() {
  if [[ $(basename "${COMPOSE_FILE:?err}") == "docker-compose.cuda-squash.build.yaml" ]]; then

    # e.g., dustynv/pytorch:2.1-r35.2.1
    DOCKER_IMG="${DEPENDENCIES_BASE_IMAGE:?err}:${DEPENDENCIES_BASE_IMAGE_TAG:?err}"
    n2st::print_msg "Pulling DOCKER_IMG=${DOCKER_IMG}"

    # shellcheck disable=SC2046
    docker pull "${DOCKER_IMG}" \
      && export $(docker inspect --format='{{range .Config.Env}}{{println .}}{{end}}' "${DOCKER_IMG}" \
        | grep \
          -e ^CUDA \
          -e ^NVIDIA \
          -e ^ROS \
          -e ^TORCH \
          -e ^LD_LIBRARY_PATH= \
          -e ^PATH= \
          -e ^RMW_IMPLEMENTATION= \
          -e ^LD_PRELOAD= \
          -e ^OPENBLAS_CORETYPE= \
          -e ^PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION= \
          -e ^TENSORBOARD_PORT= \
          -e ^JUPYTER_PORT= \
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
        | sed 's;^TENSORBOARD_PORT;BASE_IMG_ENV_TENSORBOARD_PORT;' \
        | sed 's;^JUPYTER_PORT;BASE_IMG_ENV_JUPYTER_PORT;' \
       )

    # ....Special step for handling nvidia/pytorch container conda install.........................
    if [[ ${DEPENDENCIES_BASE_IMAGE} == "nvcr.io/nvidia/pytorch" ]] || [[ ${DEPENDENCIES_BASE_IMAGE} == "nvidia/cuda" ]]; then
      export BASE_IMG_ENV_CUDA_HOME="/usr/local/cuda"
      # (CRITICAL) ToDo: assessment >> next line ↓↓
      export BASE_IMG_ENV_PATH="/usr/bin:${BASE_IMG_ENV_PATH:?err}"
    fi

    n2st::print_msg "Passing the following environment variable from ${MSG_DIMMED_FORMAT}${DEPENDENCIES_BASE_IMAGE}:${DEPENDENCIES_BASE_IMAGE_TAG}${MSG_END_FORMAT} to ${MSG_DIMMED_FORMAT}${DN_HUB:?err}/dockerized-norlab-base-image:${DN_IMAGE_TAG:?err}${MSG_END_FORMAT}:
      ${MSG_DIMMED_FORMAT}\n$(printenv | grep -e BASE_IMG_ENV_ | sed 's;BASE_IMG_ENV_;    ;')
      ${MSG_END_FORMAT}"
  else
    n2st::print_msg "Skiping base image environment variable fetching"
  fi
}


# ===============================================================================================
# Pre docker command execution callback
#
# Usage:
#   $ source dn_callback_execute_compose_pre.bash && callback_execute_compose_pre
#
# Globals:
#   Read COMPOSE_FILE
#   Read DEPENDENCIES_BASE_IMAGE
#   Read DEPENDENCIES_BASE_IMAGE_TAG
#
# =================================================================================================
function dn::callback_execute_compose_pre() {

  # ....OS version convertion......................................................................
  if [[ ${OS_NAME:?err} == ubuntu ]]; then
    if [[ ${TAG_OS_VERSION} == jammy ]]; then
      export UBUNTU_VERSION_MAJOR=22
    elif [[ ${TAG_OS_VERSION} == focal ]]; then
      export UBUNTU_VERSION_MAJOR=20
    elif [[ ${TAG_OS_VERSION} =~ "r35".* ]]; then
      export UBUNTU_VERSION_MAJOR=20
    elif [[ ${TAG_OS_VERSION} =~ "r36".* ]]; then
      export UBUNTU_VERSION_MAJOR=22
    else
      n2st::print_msg_error_and_exit "TAG_OS_VERSION=${TAG_OS_VERSION} not suported yet by base image callback"
    fi
  elif [[ ${OS_NAME} == l4t ]]; then
    if [[ ${TAG_OS_VERSION} =~ "r35".* ]]; then
      export UBUNTU_VERSION_MAJOR=20
      export L4T_CUDA_VERSION=11.4.3
    elif [[ ${TAG_OS_VERSION} =~ "r36".* ]]; then
      export UBUNTU_VERSION_MAJOR=22
      export L4T_CUDA_VERSION=12.2.2
    else
      n2st::print_msg_error_and_exit "TAG_OS_VERSION=${TAG_OS_VERSION} not suported yet by base image callback"
    fi

    # e.g., dustynv/pytorch:2.1-r35.2.1
    DOCKER_IMG="${DEPENDENCIES_BASE_IMAGE:?err}:${DEPENDENCIES_BASE_IMAGE_TAG:?err}"
    FETCH_CUDA_VERSION_MAJOR_MINOR=$( docker pull "${DOCKER_IMG}" && docker run --privileged -it --rm "${DOCKER_IMG}" nvcc --version | grep "release" | awk '{print $5}' | sed 's/,//')
    if [[ ${FETCH_CUDA_VERSION_MAJOR_MINOR} =~ ${L4T_CUDA_VERSION} ]]; then
        n2st::print_msg_error_and_exit "Cuda version for multiaarch l4t mimic image do not match!"
    fi

  fi

  # ....Reformat mimic of dustynv base image name and tag..........................................
  if [[ ${DEPENDENCIES_BASE_IMAGE} =~ "dustynv/".* ]]; then
    # Note: L4T mimic base image 'nvidia/cuda:12.3.1-runtime-ubuntu20.04' (ref https://hub.docker.com/r/nvidia/cuda),
    export MIMIC_DEPENDENCIES_BASE_IMAGE="nvidia/cuda"
    export MIMIC_DEPENDENCIES_BASE_IMAGE_TAG="${L4T_CUDA_VERSION}-runtime-ubuntu${UBUNTU_VERSION_MAJOR}.04"

  fi

  # ....Reformat nvidia/pytorch base image tag.....................................................
  if [[ ${DEPENDENCIES_BASE_IMAGE} == "nvcr.io/nvidia/pytorch" ]]; then
    export DEPENDENCIES_BASE_IMAGE_TAG="${UBUNTU_VERSION_MAJOR}.${BASE_IMG_TAG_PREFIX}"
  fi

  # ....Reformat nvidia/cuda base image tag........................................................
  if [[ ${DEPENDENCIES_BASE_IMAGE} == "nvidia/cuda" ]]; then
    export DEPENDENCIES_BASE_IMAGE_TAG="${BASE_IMG_TAG_PREFIX}${UBUNTU_VERSION_MAJOR}.04"
  fi

  # ....Export image tag for squashed base image use...............................................
  export DN_IMAGE_TAG_NO_ROS="DN-${REPOSITORY_VERSION}-${DN_IMAGE_TAG_END}"

  # ....Fetch base image environment variables.....................................................
  if [[ ! -d ${NBS_COMPOSE_DIR:?err} ]]; then
    n2st::print_msg_error_and_exit "The directory ${NBS_COMPOSE_DIR} is unreachable"
  fi

  # ....Execute cuda squash base image logic.......................................................
  dn::cuda_squash_image_logic

  n2st::print_msg_warning "DEPENDENCIES_BASE_IMAGE=${DEPENDENCIES_BASE_IMAGE}"
  n2st::print_msg_warning "DEPENDENCIES_BASE_IMAGE_TAG=${DEPENDENCIES_BASE_IMAGE_TAG}"
  n2st::print_msg_warning "DN_IMAGE_TAG_NO_ROS=${DN_IMAGE_TAG_NO_ROS}"
}

