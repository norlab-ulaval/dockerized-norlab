#!/bin/bash

PCK_VERSION=$(pip3 list --format freeze)
SP="    "

echo

echo -e "In-container informations:"
#${DN_CONTAINER_NAME} container info
echo -e "\033[1;37m
${SP}DN container name:           ${DN_CONTAINER_NAME}
${SP}DN host name:                $(hostname)
${SP}DN image architecture:       ${DN_IMAGE_ARCHITECTURE}
${SP}DN project src path:         ${DN_DEV_WORKSPACE}/src/${DN_TARGET_PROJECT_SRC_REPO}
${SP}DN target project repo:      ${DN_TARGET_PROJECT_SRC_REPO}
${SP}DN activate powerline promt: ${DN_ACTIVATE_POWERLINE_PROMT}
${SP}
${SP}ROS distro:                  ${ROS_DISTRO}
${SP}ROS package:                 ${ROS_PKG}
${SP}ROS python version:          ${ROS_PYTHON_VERSION}
${SP}ROS domain id:               ${ROS_DOMAIN_ID}
${SP}ROS master uri:              ${ROS_MASTER_URI}
${SP}
${SP}python3 version:             ${DS_PYTHON3_VERSION}
${SP}numpy version:               $(echo "${PCK_VERSION}" | grep numpy | sed 's/numpy==//g')
${SP}pyTorch version:             $(echo "${PCK_VERSION}" | grep -w torch | sed 's/torch==//g')
${SP}torchvision version:         $(echo "${PCK_VERSION}" | grep -w torchvision | sed 's/torchvision==//g')
${SP}numba version:               $(echo "${PCK_VERSION}" | grep numba | sed 's/numba==//g')
${SP}LLVMlite version:            $(echo "${PCK_VERSION}" | grep llvmlite | sed 's/llvmlite==//g')
\033[0m"
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
${SP}$ ssh -p ${DS_PYCHARM_DEV_SERVER_PORT} ${DS_PYCHARM_DEV_USER}@$(hostname -I | awk '{print $1}')
${SP}$ sftp -P ${DS_PYCHARM_DEV_SERVER_PORT} openssh-$(hostname -I | awk '{print $1}')
${SP}$ scp -P ${DS_PYCHARM_DEV_SERVER_PORT} /path/to/source ${DS_PYCHARM_DEV_USER}@$(hostname -I | awk '{print $1}'):/target/dir/
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

# ToDo: Validate output
echo -e "If you're running a python interpreter in remote development, dont forget to add the python path pointing to the ROS package:
\033[1;37m
${SP} PYTHONPATH=${PYTHONPATH}
\033[0m
"
