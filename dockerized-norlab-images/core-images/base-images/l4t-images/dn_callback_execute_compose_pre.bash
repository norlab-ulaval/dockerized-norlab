#!/bin/bash

declare -x MIMIC_DEPENDENCIES_BASE_IMAGE
declare -x MIMIC_DEPENDENCIES_BASE_IMAGE_TAG
declare -x UBUNTU_VERSION_MAJOR
declare -x L4T_CUDA_VERSION
declare -x DN_IMAGE_TAG_NO_ROS

# ===============================================================================================
# Pre docker command execution callback
#
# Usage:
#   $ source dn_callback_execute_compose_pre.bash && callback_execute_compose_pre
#
# Globals:
#   Read OS_NAME
#   Read TAG_OS_VERSION
#   Read REPOSITORY_VERSION
#   Read DN_IMAGE_TAG_END
#   Read DN_IMAGE_TAG
#   Read NBS_COMPOSE_DIR
#   Read DEPENDENCIES_BASE_IMAGE
#   Read DEPENDENCIES_BASE_IMAGE_TAG
#
# =================================================================================================
function dn::callback_execute_compose_pre() {

    n2st::print_msg "Pre-condition checks..."
    {
        test -n "${OS_NAME:?'Env variable need to be set and non-empty.'}" && \
        test -n "${TAG_OS_VERSION:?'Env variable need to be set and non-empty.'}" && \
        test -n "${REPOSITORY_VERSION:?'Env variable need to be set and non-empty.'}" && \
        test -n "${DN_IMAGE_TAG_END:?'Env variable need to be set and non-empty.'}" && \
        test -n "${DN_IMAGE_TAG:?'Env variable need to be set and non-empty.'}" && \
        test -n "${NBS_COMPOSE_DIR:?'Env variable need to be set and non-empty.'}" && \
        test -n "${DEPENDENCIES_BASE_IMAGE:?'Env variable need to be set and non-empty.'}" && \
        test -n "${DEPENDENCIES_BASE_IMAGE_TAG:?'Env variable need to be set and non-empty.'}" ;
    } || return 1

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
    local docker_img="${DEPENDENCIES_BASE_IMAGE:?err}:${DEPENDENCIES_BASE_IMAGE_TAG:?err}"
    n2st::print_msg "Pulling docker_img=${docker_img}..."
    docker pull --platform="linux/arm64" "${docker_img}"

    local fetch_cuda_version_major_minor
    fetch_cuda_version_major_minor=$( docker run --privileged -it --rm "${docker_img}" nvcc --version | grep "release" | awk '{print $5}' | sed 's/,//')
    if [[ ${fetch_cuda_version_major_minor} =~ ${L4T_CUDA_VERSION} ]]; then
        n2st::print_msg_error_and_exit "Cuda version for multiaarch l4t mimic image do not match!"
    fi

    # ....Reformat mimic of dustynv base image name and tag........................................
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
      local tensorrt_release
      if [[ "${UBUNTU_VERSION_MAJOR}" == "20" ]]; then
        # TensorRT Release 23.04 -> last Ubuntu 20.04 release -> python 3.8
        #   https://docs.nvidia.com/deeplearning/frameworks/container-release-notes/index.html#rel-23-04
        tensorrt_release=23.04
      elif [[ "${UBUNTU_VERSION_MAJOR}" == "22" ]]; then
        # TensorRT Release 24.10 -> last Ubuntu 22.04 release -> python 3.10
        #   https://docs.nvidia.com/deeplearning/frameworks/container-release-notes/index.html#rel-24-10)
        tensorrt_release=24.10
      elif [[ "${UBUNTU_VERSION_MAJOR}" == "24" ]]; then
        # Latest available at the time
        tensorrt_release=25.06
      fi

      # 💎 Run 'dockerized-norlab-scripts/build_script/dn_push_tensorrt_to_norlab_dockerhub.bash'
      # manualy to fetch tensorrt images from nvidia and push them to norlab dockerhub domain.
      export MIMIC_DEPENDENCIES_BASE_IMAGE="norlabulaval/nvidia-tensorrt"
      export MIMIC_DEPENDENCIES_BASE_IMAGE_TAG="${tensorrt_release}-py3"

      # Quick-hack: Pre-pull tensorrt to prevent unauthorized access error
      #docker pull --platform=linux/amd64 "${MIMIC_DEPENDENCIES_BASE_IMAGE}:${MIMIC_DEPENDENCIES_BASE_IMAGE_TAG}"
    fi

    # ....Export image tag for squashed base image use.............................................
    export DN_IMAGE_TAG_NO_ROS="DN-${REPOSITORY_VERSION}-${DN_IMAGE_TAG_END}"

    # ....Fetch base image environment variables...................................................
    if [[ ! -d ${NBS_COMPOSE_DIR:?err} ]]; then
      n2st::print_msg_error_and_exit "The directory ${NBS_COMPOSE_DIR} is unreachable"
    fi

    # ....Execute cuda squash base image logic.....................................................
    if [[ ${DEPENDENCIES_BASE_IMAGE} == "dustynv/l4t-pytorch" ]]; then
      # Fetch env var, from a base image, to be passed to a scratch image in order to buld a mimic
      # of the base image but in a different architecture.
      #
      # Base image e.g.,
      #   - dustynv/pytorch:2.1-r35.2.1
      #   - dustynv/l4t-pytorch:r36.4.0
      #
      # Introspection
      # $ docker inspect --format='{{range .Config.Env}}{{println .}}{{end}}' "dustynv/l4t-pytorch:r36.4.0"

      local img_env_var=()
      while IFS='' read -r line; do
        img_env_var+=("$line")
      done < <( docker inspect --format='{{range .Config.Env}}{{println .}}{{end}}' "${docker_img}" \
          | grep \
            -e '^NVIDIA_' \
            -e '^CUDA_' \
            -e '^CMAKE_CUDA_COMPILER=' \
            -e '^NVCC_' \
            -e '^TORCH_' \
            -e '^ROS_' \
            -e '^LD_LIBRARY_PATH=' \
            -e '^PATH=' \
            -e '^RMW_IMPLEMENTATION=' \
            -e '^LD_PRELOAD=' \
            -e '^OPENBLAS_CORETYPE=' \
          | sed 's;^CUDA_HOME;L4T_BASE_IMG_ENV_CUDA_HOME;' \
          | sed 's;^NVIDIA;L4T_BASE_IMG_ENV_NVIDIA;' \
          | sed 's;^PATH;L4T_BASE_IMG_ENV_PATH;' \
          | sed 's;^LD_LIBRARY_PATH;L4T_BASE_IMG_ENV_LD_LIBRARY_PATH;' \
          | sed 's;^ROS;L4T_BASE_IMG_ENV_ROS;' \
          | sed 's;^RMW_IMPLEMENTATION;L4T_BASE_IMG_ENV_RMW_IMPLEMENTATION;' \
          | sed 's;^LD_PRELOAD;L4T_BASE_IMG_ENV_LD_PRELOAD;' \
          | sed 's;^OPENBLAS_CORETYPE;L4T_BASE_IMG_ENV_OPENBLAS_CORETYPE;' \
          | sed 's;^TORCH_HOME;L4T_BASE_IMG_ENV_TORCH_HOME;' \
          | sed 's;^TORCH_NVCC_FLAGS=\(.*\);L4T_BASE_IMG_ENV_TORCH_NVCC_FLAGS="\1";' \
          | sed 's;^TORCH_CUDA_ARCH_LIST;L4T_BASE_IMG_ENV_TORCH_CUDA_ARCH_LIST;' \
          | sed 's;^NVCC_PATH;L4T_BASE_IMG_ENV_NVCC_PATH;' \
          | sed 's;^CUDA_BIN_PATH;L4T_BASE_IMG_ENV_CUDA_BIN_PATH;' \
          | sed 's;^CMAKE_CUDA_COMPILER;L4T_BASE_IMG_ENV_CMAKE_CUDA_COMPILER;' \
          | sed 's;^CUDAARCHS;L4T_BASE_IMG_ENV_CUDAARCHS;' \
          | sed 's;^CUDACXX;L4T_BASE_IMG_ENV_CUDACXX;' \
          | sed 's;^CUDA_TOOLKIT_ROOT_DIR;L4T_BASE_IMG_ENV_CUDA_TOOLKIT_ROOT_DIR;' \
          | sed 's;^CUDA_NVCC_EXECUTABLE;L4T_BASE_IMG_ENV_CUDA_NVCC_EXECUTABLE;' \
          | sed 's;^CUDA_ARCHITECTURES;L4T_BASE_IMG_ENV_CUDA_ARCHITECTURES;' \
         )

      local img_env_var_size=${#img_env_var[@]}
      echo -e "Exporting ${img_env_var_size} environment variables from ${docker_img}..."
      if [[ img_env_var_size -eq 1 ]]; then
        n2st::print_msg_error "Problem while fetching env var from dustynv/l4t-pytorch"
        return 1
      else
        export "${img_env_var[@]}"
      fi

      n2st::print_msg "Passing the following environment variable from ${MSG_DIMMED_FORMAT}${DEPENDENCIES_BASE_IMAGE}:${DEPENDENCIES_BASE_IMAGE_TAG}${MSG_END_FORMAT} to ${MSG_DIMMED_FORMAT}${DN_HUB:?err}/dockerized-norlab-base-image:${DN_IMAGE_TAG:?err}${MSG_END_FORMAT}:
        ${MSG_DIMMED_FORMAT}\n$(printenv | grep -e L4T_BASE_IMG_ENV_ | sed 's;L4T_BASE_IMG_ENV_;    ;')
        ${MSG_END_FORMAT}"
    else
      n2st::print_msg "Skiping base image environment variable fetching"
    fi

    # ====End======================================================================================
    n2st::print_msg "Execute compose callback pre output
      ${MSG_DIMMED_FORMAT}
      DEPENDENCIES_BASE_IMAGE=${DEPENDENCIES_BASE_IMAGE}
      DEPENDENCIES_BASE_IMAGE_TAG=${DEPENDENCIES_BASE_IMAGE_TAG}
      DN_IMAGE_TAG_NO_ROS=${DN_IMAGE_TAG_NO_ROS}
      MIMIC_DEPENDENCIES_BASE_IMAGE=${MIMIC_DEPENDENCIES_BASE_IMAGE}
      MIMIC_DEPENDENCIES_BASE_IMAGE_TAG=${MIMIC_DEPENDENCIES_BASE_IMAGE_TAG}
      ${MSG_END_FORMAT}"

  else
    n2st::print_msg_error_and_exit "OS_NAME=${OS_NAME} not suported yet by base image callback"
  fi

  return 0
}

