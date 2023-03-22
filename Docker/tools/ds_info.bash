#!/bin/bash

PCK_VERSION=$(pip3 list --format freeze)
SP="  "

#${DS_CONTAINER_NAME} container info
echo -e "\033[1;37m
${SP}ROS distro:              ${ROS_DISTRO}
${SP}python3 version:         ${DS_PYTHON3_VERSION}\033[0m
${SP}Numpy version:           $(echo "${PCK_VERSION}" | grep numpy | sed 's/numpy==//g')
${SP}PyTorch version:         $(echo "${PCK_VERSION}" | grep torch | sed 's/torch==//g')
${SP}Numba version:           $(echo "${PCK_VERSION}" | grep numba | sed 's/numba==//g')
${SP}LLVMlite version:        $(echo "${PCK_VERSION}" | grep llvmlite | sed 's/llvmlite==//g')
${SP}ROS package:             ${DS_ROS_PKG}
${SP}ROS python version:      ${ROS_PYTHON_VERSION}
${SP}ROS master uri:          ${ROS_MASTER_URI}
${SP}DS image architecture:   ${DS_IMAGE_ARCHITECTURE}
${SP}Project src code path:   ${DS_DEV_WORKSPACE}/src/${DS_TARGET_PROJECT_SRC_REPO}
"
#${SP}PyCuda version:          $(echo "${PCK_VERSION}" | grep pycuda | sed 's/pycuda==//g')
