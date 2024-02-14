#!/bin/bash

declare -x CONVERTED_TAG_OS_VERSION

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

  # ....OS version convertion......................................................................
  if [[ ${OS_NAME} == l4t ]]; then
    if [[ ${TAG_OS_VERSION} =~ "r35".* ]]; then
      export CONVERTED_TAG_OS_VERSION=20
      export L4T_CUDA_VERSION=11.4.3
    elif [[ ${TAG_OS_VERSION} =~ "r36".* ]]; then
      export CONVERTED_TAG_OS_VERSION=22
      export L4T_CUDA_VERSION=12.2.2
    else
      n2st::print_msg_error_and_exit "TAG_OS_VERSION=${TAG_OS_VERSION} not suported yet by base image callback"
    fi

    # ex: dustynv/pytorch:2.1-r35.2.1
    DOCKER_IMG="${DEPENDENCIES_BASE_IMAGE:?err}:${DEPENDENCIES_BASE_IMAGE_TAG:?err}"
    docker pull --platform="linux/arm64" "${DOCKER_IMG}"
    FETCH_CUDA_VERSION_MAJOR_MINOR=$( docker run --privileged -it --rm "${DOCKER_IMG}" nvcc --version | grep "release" | awk '{print $5}' | sed 's/,//')
    if [[ ${FETCH_CUDA_VERSION_MAJOR_MINOR} =~ ${L4T_CUDA_VERSION} ]]; then
        n2st::print_msg_error_and_exit "Cuda version for multiaarch l4t mimic image do not match!"
    fi

  else
    n2st::print_msg_error_and_exit "OS_NAME=${OS_NAME} not suported yet by base image callback"
  fi

  # ....Reformat mimic of dustynv base image name and tag..........................................
  if [[ ${DEPENDENCIES_BASE_IMAGE} =~ "dustynv/".* ]]; then
#    # "cuda runtime" as base for L4T image mimic
#    #  - base image 'nvidia/cuda:12.3.1-runtime-ubuntu20.04'
#    #  - ref https://hub.docker.com/r/nvidia/cuda
#    export MIMIC_DEPENDENCIES_BASE_IMAGE="nvidia/cuda"
#    export MIMIC_DEPENDENCIES_BASE_IMAGE_TAG="${L4T_CUDA_VERSION}-runtime-ubuntu${CONVERTED_TAG_OS_VERSION}.04"

    # "tensorrt" as base for L4T image mimic
    #   - Comme with pycuda and tensorrt installed
    #   - base image 'nvcr.io/nvidia/tensorrt:20.12-py3'
    #   - ref https://catalog.ngc.nvidia.com/orgs/nvidia/containers/tensorrt/tags
    export MIMIC_DEPENDENCIES_BASE_IMAGE="nvcr.io/nvidia/tensorrt"
    export MIMIC_DEPENDENCIES_BASE_IMAGE_TAG="${CONVERTED_TAG_OS_VERSION}.12-py3"

  fi

  # ....Export image tag for squashed base image use...............................................
  export DN_IMAGE_TAG_NO_ROS="DN-${REPOSITORY_VERSION}-${DN_IMAGE_TAG_END}"

  # ....Fetch base image environment variables.....................................................
  if [[ ! -d ${NBS_COMPOSE_DIR:?err} ]]; then
    n2st::print_msg_error_and_exit "The directory ${NBS_COMPOSE_DIR} is unreachable"
  fi

  # ....Execute cuda squash base image logic.......................................................
  if [[ ${DEPENDENCIES_BASE_IMAGE} == "dustynv/l4t-pytorch" ]]; then

    # ex: dustynv/pytorch:2.1-r35.2.1
    DOCKER_IMG="${DEPENDENCIES_BASE_IMAGE:?err}:${DEPENDENCIES_BASE_IMAGE_TAG:?err}"
    n2st::print_msg "Pulling DOCKER_IMG=${DOCKER_IMG}"

    # shellcheck disable=SC2046
    docker pull --platform="linux/arm64" "${DOCKER_IMG}" \
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
          -e TORCH_HOME= \
        | sed 's;^CUDA_HOME;BASE_IMG_ENV_CUDA_HOME;' \
        | sed 's;^NVIDIA_;BASE_IMG_ENV_NVIDIA_;' \
        | sed 's;^PATH;BASE_IMG_ENV_PATH;' \
        | sed 's;^LD_LIBRARY_PATH;BASE_IMG_ENV_LD_LIBRARY_PATH;' \
        | sed 's;^ROS_;BASE_IMG_ENV_ROS_;' \
        | sed 's;^RMW_IMPLEMENTATION;BASE_IMG_ENV_RMW_IMPLEMENTATION;' \
        | sed 's;^LD_PRELOAD;BASE_IMG_ENV_LD_PRELOAD;' \
        | sed 's;^OPENBLAS_CORETYPE;BASE_IMG_ENV_OPENBLAS_CORETYPE;' \
        | sed 's;^TORCH_HOME;BASE_IMG_ENV_TORCH_HOME;' \
       )

    n2st::print_msg "Passing the following environment variable from ${MSG_DIMMED_FORMAT}${DEPENDENCIES_BASE_IMAGE}:${DEPENDENCIES_BASE_IMAGE_TAG}${MSG_END_FORMAT} to ${MSG_DIMMED_FORMAT}${DN_HUB:?err}/dockerized-norlab-base-image:${DN_IMAGE_TAG:?err}${MSG_END_FORMAT}:
      ${MSG_DIMMED_FORMAT}\n$(printenv | grep -e BASE_IMG_ENV_ | sed 's;BASE_IMG_ENV_;    ;')
      ${MSG_END_FORMAT}"
  else
    n2st::print_msg "Skiping base image environment variable fetching"
  fi

  # ====End========================================================================================
  n2st::print_msg "Execute compose callback pre output
    ${MSG_DIMMED_FORMAT}
    DEPENDENCIES_BASE_IMAGE=${DEPENDENCIES_BASE_IMAGE}
    DEPENDENCIES_BASE_IMAGE_TAG=${DEPENDENCIES_BASE_IMAGE_TAG}
    DN_IMAGE_TAG_NO_ROS=${DN_IMAGE_TAG_NO_ROS}
    MIMIC_DEPENDENCIES_BASE_IMAGE=${MIMIC_DEPENDENCIES_BASE_IMAGE}
    MIMIC_DEPENDENCIES_BASE_IMAGE_TAG=${MIMIC_DEPENDENCIES_BASE_IMAGE_TAG}
    ${MSG_END_FORMAT}"
}

