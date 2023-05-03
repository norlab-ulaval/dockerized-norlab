#!/bin/bash

# Load environment variable from file
#set -o allexport; source "${DN_PATH}/.env.prompt"; set +o allexport
set -o allexport; source .env.prompt; set +o allexport

# DN_IMAGE_ARCHITECTURE: arm64-l4t, arm64-darwin, x86
# ARCH: aarch64, arm64, x86_64
# OS: Linux, Darwin

if [[ $(uname -m) == "aarch64" ]]; then
  if [[ -n $(uname -r | grep tegra) ]]; then
    export DN_IMAGE_ARCHITECTURE='arm64-l4t'

#    # See https://github.com/RedLeader962/Dockerized-SNOW/blob/8a55ccbdb75d887d7e8b3e112afc6746286f99a9/ds_build_melodic_python3.bash
#    if [[ "$BASE_IMG_VERSION" == "" ]]; then
#      BASE_IMG_VERSION="r32.6.1"
#    fi
#    BASE_IMG_ARG=" --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-base:${BASE_IMG_VERSION}"

  else
    echo -e "${DS_MSG_ERROR} Unsuported OS for aarch64 processor"
  fi
elif [[ $(uname -m) == "arm64" ]]; then
  if [[ $(uname) == "Darwin" ]]; then
    export DN_IMAGE_ARCHITECTURE='arm64-darwin'

#    # See https://github.com/RedLeader962/Dockerized-SNOW/blob/8a55ccbdb75d887d7e8b3e112afc6746286f99a9/ds_build_melodic_python3.bash
#    if [[ "$BASE_IMG_VERSION" == "" ]]; then
#      BASE_IMG_VERSION="18.04"
#    fi
#    BASE_IMG_ARG=" --build-arg BASE_IMAGE=arm64v8/ubuntu:${BASE_IMG_VERSION}"
#    BASE_IMG_VERSION="ubuntu${BASE_IMG_VERSION}"

  else
    echo -e "${DS_MSG_ERROR} Unsuported OS for arm64 processor"
  fi
elif [[ $(uname -m) == "x86_64" ]]; then
  if [[ $(uname) == "Linux" ]]; then
    export DN_IMAGE_ARCHITECTURE='x86'

#    # See https://github.com/RedLeader962/Dockerized-SNOW/blob/8a55ccbdb75d887d7e8b3e112afc6746286f99a9/ds_build_melodic_python3.bash
#    BASE_IMG_VERSION="ubuntu18.04"
#    BASE_IMG_ARG=" --build-arg BASE_IMAGE=nvcr.io/nvidia/cudagl:11.4.0-devel-${BASE_IMG_VERSION}"

  else
    echo -e "${DS_MSG_ERROR} Unsuported OS for x86 processor"
  fi
else
  echo -e "${DS_MSG_ERROR} Unsuported processor architecture"
fi

echo -e "${DS_MSG_BASE} which DN_IMAGE_ARCHITECTURE env? $DN_IMAGE_ARCHITECTURE"
