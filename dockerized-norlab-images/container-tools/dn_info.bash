#!/bin/bash

PCK_VERSION=$(pip3 list --format freeze)
SP="    "

TMP_CWD=$(pwd)
cd /dockerized-norlab/utilities/norlab-shell-script-tools/src/utility_scripts/
source ./which_python_version.bash
source ./which_architecture_and_os.bash
DN_PYTHON3_VERSION=${PYTHON3_VERSION?err}
DN_IMAGE_ARCHITECTURE=${IMAGE_ARCH_AND_OS:?err}
cd "$TMP_CWD"

echo

echo -e "In-container informations:"
#${DN_CONTAINER_NAME} container info
echo -e "\033[1;37m
${SP}DN container name:           ${DN_CONTAINER_NAME}
${SP}DN user:                     $(whoami)
${SP}DN host name:                $(hostname)
${SP}DN image architecture:       ${DN_IMAGE_ARCHITECTURE}
${SP}DN activate powerline promt: ${DN_ACTIVATE_POWERLINE_PROMT}
${SP}
${SP}DN target project repo:      https://github.com/${DN_PROJECT_GIT_DOMAIN}/${DN_PROJECT_GIT_NAME}.git
${SP}DN project src path:         ${DN_DEV_WORKSPACE}/src/${DN_PROJECT_GIT_NAME}
${SP}
${SP}ROS distro:                  ${ROS_DISTRO}
${SP}ROS domain id:               ${ROS_DOMAIN_ID}
${SP}ROS container workspace:     ${DN_DEV_WORKSPACE}
${SP}
${SP}python3 version:             ${DN_PYTHON3_VERSION}
${SP}numpy version:               $(echo "${PCK_VERSION}" | grep numpy | sed 's/numpy==//g')
${SP}pyTorch version:             $(echo "${PCK_VERSION}" | grep -w torch | sed 's/torch==//g')
${SP}torchvision version:         $(echo "${PCK_VERSION}" | grep -w torchvision | sed 's/torchvision==//g')
${SP}numba version:               $(echo "${PCK_VERSION}" | grep numba | sed 's/numba==//g')
${SP}LLVMlite version:            $(echo "${PCK_VERSION}" | grep llvmlite | sed 's/llvmlite==//g')
\033[0m"
#${SP}ROS python version:          ${ROS_PYTHON_VERSION}
#${SP}PyCuda version:          $(echo "${PCK_VERSION}" | grep pycuda | sed 's/pycuda==//g')

echo -e "In-container available alias:
\033[1;37m
${SP}$ dn_info
${SP}$ dn_python3_check
${SP}$ dn_gym_check
${SP}$ dn_ros2_rebuild_dev_workspace
${SP}$ dn_fetch_ros_env
\033[0m
"

echo -e "IDE remote development workflow › to connect to the container internal ssh server:
\033[1;37m
${SP}$ ssh -p ${DN_SSH_SERVER_PORT} ${DN_SSH_SERVER_USER}@$(hostname -I | awk '{print $1}')
${SP}$ sftp -P ${DN_SSH_SERVER_PORT} openssh-$(hostname -I | awk '{print $1}')
${SP}$ scp -P ${DN_SSH_SERVER_PORT} /path/to/source ${DN_SSH_SERVER_USER}@$(hostname -I | awk '{print $1}'):/target/dir/
\033[0m
"

# (NICE TO HAVE) ToDo: Add >> procedure for configuring .env file
echo -e "Terminal prompt › The default Dockerized-NorLab prompt require that\033[1;37m Powerline-status\033[0m or\033[1;37m Powerline10k\033[0m be installed on the host terminal. To change to a minimal prompt, either set permanently the ENV variable in\033[1;37m docker-compose.<spec>.run.yaml\033[0m:
${SP}
${SP}services:
${SP}  develop: # the service name
${SP}\033[1;37m    environment:
${SP}      - DN_ACTIVATE_POWERLINE_PROMT=false
\033[0m
or pass the following flag to \033[1;37mdn_attach\033[0m when connecting to a running container:
\033[1;37m
${SP}$ dn_attach --env=\"DN_ACTIVATE_POWERLINE_PROMT=false\" <the-running-container-name>
\033[0m
"

# ToDo: Validate output › PYTHONPATH is not accessible the first time dn_info is called
echo -e "If you're running a python interpreter in remote development, dont forget to add the python path pointing to the ROS package:
\033[1;37m
${SP} PYTHONPATH=${PYTHONPATH}
\033[0m"
