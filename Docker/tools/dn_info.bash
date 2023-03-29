#!/bin/bash

PCK_VERSION=$(pip3 list --format freeze)
SP="    "

echo

echo -e "Container informations:"
#${DN_CONTAINER_NAME} container info
echo -e "\033[1;37m
${SP}DN Container name:           ${DN_CONTAINER_NAME}
${SP}DN Image architecture:       ${DS_IMAGE_ARCHITECTURE}
${SP}DN Target project repo:      ${DN_TARGET_PROJECT_SRC_REPO}
${SP}DN Activate powerline promt: ${DN_ACTIVATE_POWERLINE_PROMT}
${SP}DN Project src code path:    ${DS_DEV_WORKSPACE}/src/${DN_TARGET_PROJECT_SRC_REPO}
${SP}
${SP}ROS distro:                  ${ROS_DISTRO}
${SP}ROS package:                 ${DS_ROS_PKG}
${SP}ROS python version:          ${ROS_PYTHON_VERSION}
${SP}ROS domain id:               ${ROS_DOMAIN_ID}
${SP}ROS master uri:              ${ROS_MASTER_URI}
${SP}
${SP}python3 version:             ${DS_PYTHON3_VERSION}
${SP}Numpy version:               $(echo "${PCK_VERSION}" | grep numpy | sed 's/numpy==//g')
${SP}PyTorch version:             $(echo "${PCK_VERSION}" | grep -w torch | sed 's/torch==//g')
${SP}Numba version:               $(echo "${PCK_VERSION}" | grep numba | sed 's/numba==//g')
${SP}LLVMlite version:            $(echo "${PCK_VERSION}" | grep llvmlite | sed 's/llvmlite==//g')
\033[0m"
#${SP}PyCuda version:          $(echo "${PCK_VERSION}" | grep pycuda | sed 's/pycuda==//g')

echo -e "In container available alias:
\033[1;37m
${SP}dn_info
${SP}dn_python3_check
${SP}dn_ros2_rebuild_dev_workspace
${SP}dn_fetch_ros_env
\033[0m
"

echo -e "\033[1;37mIDE remote development workflow\033[0m: to connect to the container internal ssh server:
\033[1;37m
${SP}$ ssh -p ${DS_PYCHARM_DEV_SERVER_PORT} ${DS_PYCHARM_DEV_USER}@$(hostname -I | awk '{print $1}')
${SP}$ sftp -P ${DS_PYCHARM_DEV_SERVER_PORT} openssh-$(hostname -I | awk '{print $1}')
${SP}$ scp -P ${DS_PYCHARM_DEV_SERVER_PORT} /path/to/foo ${DS_PYCHARM_DEV_USER}@$(hostname -I | awk '{print $1}'):/path/to/dest/
\033[0m
"

echo -e "\033[1;37mTerminal prompt\033[0m: To change to a minimal prompt or if your terminal does not have \033[1;37m Powerline-status \033[0m or \033[1;37m Powerline10k \033[0m installed:
${SP}# Execute the following line in the container
\033[1;37m
${SP}$ export DN_ACTIVATE_POWERLINE_PROMT=false
${SP}$ source ~/.bashrc
\033[0m
"

