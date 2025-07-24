#!/bin/bash

declare -x MIMIC_DEPENDENCIES_BASE_IMAGE
declare -x MIMIC_DEPENDENCIES_BASE_IMAGE_TAG
declare -x UBUNTU_VERSION_MAJOR

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
  if [[ ${OS_NAME} == l4t ]]; then
    if [[ ${TAG_OS_VERSION} =~ "r35".* ]]; then
      export UBUNTU_VERSION_MAJOR=20
      export L4T_CUDA_VERSION=11.4.3
    elif [[ ${TAG_OS_VERSION} =~ "r36".* ]]; then
      export UBUNTU_VERSION_MAJOR=22
      export L4T_CUDA_VERSION=12.2.2
    else
      n2st::print_msg_error_and_exit "TAG_OS_VERSION=${TAG_OS_VERSION} not suported yet by base image callback"
    fi

    # e.g.,
    #   - dustynv/pytorch:2.1-r35.2.1
    #   - dustynv/l4t-pytorch:r36.4.0
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
    ## "cuda runtime" as a base image for L4T image mimic
    ##  - base image 'nvidia/cuda:12.3.1-runtime-ubuntu20.04'
    ##  - ref https://hub.docker.com/r/nvidia/cuda
    #export MIMIC_DEPENDENCIES_BASE_IMAGE="nvidia/cuda"
    #export MIMIC_DEPENDENCIES_BASE_IMAGE_TAG="${L4T_CUDA_VERSION}-runtime-ubuntu${UBUNTU_VERSION_MAJOR}.04"

    # "tensorrt" as a base image for L4T image mimic
    #   - Comme with pycuda and tensorrt installed
    #   - base image 'nvcr.io/nvidia/tensorrt:20.12-py3'
    #   - ref https://catalog.ngc.nvidia.com/orgs/nvidia/containers/tensorrt/tags
    if [[ "${UBUNTU_VERSION_MAJOR}" == "20" ]]; then
      # TensorRT Release 23.04 -> last Ubuntu 20.04 release -> python 3.8
      #   https://docs.nvidia.com/deeplearning/frameworks/container-release-notes/index.html#rel-23-04
      TENSORRT_RELEASE=23.04
    elif [[ "${UBUNTU_VERSION_MAJOR}" == "22" ]]; then
      # TensorRT Release 24.10 -> last Ubuntu 22.04 release -> python 3.10
      #   https://docs.nvidia.com/deeplearning/frameworks/container-release-notes/index.html#rel-24-10)
      TENSORRT_RELEASE=24.10
    elif [[ "${UBUNTU_VERSION_MAJOR}" == "24" ]]; then
      # Latest available at the time
      TENSORRT_RELEASE=25.06
    fi
    export MIMIC_DEPENDENCIES_BASE_IMAGE="nvcr.io/nvidia/tensorrt"
    export MIMIC_DEPENDENCIES_BASE_IMAGE_TAG="${TENSORRT_RELEASE}-py3"

    # Quick-hack: Pre-pull tensorrt to prevent unauthorized access error
    #docker pull --platform=linux/amd64 "${MIMIC_DEPENDENCIES_BASE_IMAGE}:${MIMIC_DEPENDENCIES_BASE_IMAGE_TAG}"
  fi

  # ....Export image tag for squashed base image use...............................................
  export DN_IMAGE_TAG_NO_ROS="DN-${REPOSITORY_VERSION}-${DN_IMAGE_TAG_END}"

  # ....Fetch base image environment variables.....................................................
  if [[ ! -d ${NBS_COMPOSE_DIR:?err} ]]; then
    n2st::print_msg_error_and_exit "The directory ${NBS_COMPOSE_DIR} is unreachable"
  fi

  # ....Execute cuda squash base image logic.......................................................
  if [[ ${DEPENDENCIES_BASE_IMAGE} == "dustynv/l4t-pytorch" ]]; then

    # e.g., dustynv/pytorch:2.1-r35.2.1
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
        | sed 's;^TORCH_NVCC_FLAGS;BASE_IMG_ENV_TORCH_NVCC_FLAGS;' \
        | sed 's;^TORCH_CUDA_ARCH_LIST;BASE_IMG_ENV_TORCH_CUDA_ARCH_LIST;' \
        | sed 's;^CMAKE_CUDA_COMPILER;BASE_IMG_ENV_CMAKE_CUDA_COMPILER;' \
        | sed 's;^CUDA_BIN_PATH;BASE_IMG_ENV_CUDA_BIN_PATH;' \
        | sed 's;^CUDAARCHS;BASE_IMG_ENV_CUDAARCHS;' \
        | sed 's;^CUDACXX;BASE_IMG_ENV_CUDACXX;' \
        | sed 's;^CUDA_TOOLKIT_ROOT_DIR;BASE_IMG_ENV_CUDA_TOOLKIT_ROOT_DIR;' \
        | sed 's;^NVCC_PATH;BASE_IMG_ENV_NVCC_PATH;' \
        | sed 's;^CUDA_NVCC_EXECUTABLE;BASE_IMG_ENV_CUDA_NVCC_EXECUTABLE;' \
        | sed 's;^CUDA_ARCHITECTURES;BASE_IMG_ENV_CUDA_ARCHITECTURES;' \
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

