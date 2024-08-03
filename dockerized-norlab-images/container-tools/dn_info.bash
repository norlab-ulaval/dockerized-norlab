#!/bin/bash
pushd "$(pwd)" >/dev/null || exit 1

PCK_VERSION=$(pip3 list --format freeze)
SP="    "

# ToDo: validate (ref task NMO-385 sub-task NMO-485 fix: Project related env var overridden issue)
cd /dockerized-norlab/dockerized-norlab-images/container-tools || exit 1
source import_dockerized_norlab_container_tools.bash

n2st::set_which_python3_version
DN_PYTHON3_VERSION=${PYTHON3_VERSION?err}

n2st::set_which_architecture_and_os
DN_IMAGE_ARCHITECTURE=${IMAGE_ARCH_AND_OS:?err}


echo
n2st::draw_horizontal_line_across_the_terminal_window '.'
# (NICE TO HAVE) ToDo: refactor  using 'n2st::print_msg' fct
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
${SP}DN project src path:         ${DN_PROJECT_PATH}
${SP}
${SP}ROS distro:                  ${ROS_DISTRO}
${SP}ROS package:                 ${ROS_PKG}
${SP}ROS domain id:               ${ROS_DOMAIN_ID}
${SP}ROS container workspace:     ${DN_DEV_WORKSPACE}
${SP}RMW_IMPLEMENTATION:          ${RMW_IMPLEMENTATION}
${SP}
${SP}python3 version:             ${DN_PYTHON3_VERSION}
${SP}numpy version:               $(echo "${PCK_VERSION}" | grep numpy== | sed 's/numpy==//g')
${SP}pyTorch version:             $(echo "${PCK_VERSION}" | grep -w torch | sed 's/torch==//g')
${SP}torchvision version:         $(echo "${PCK_VERSION}" | grep -w torchvision | sed 's/torchvision==//g')
${SP}numba version:               $(echo "${PCK_VERSION}" | grep numba | sed 's/numba==//g')
${SP}LLVMlite version:            $(echo "${PCK_VERSION}" | grep llvmlite | sed 's/llvmlite==//g')
\033[0m"
#${SP}ROS python version:          ${ROS_PYTHON_VERSION}
#${SP}PyCuda version:          $(echo "${PCK_VERSION}" | grep pycuda | sed 's/pycuda==//g')

echo -e "In-container available alias:
\033[1;37m
$( \
    cd ${DN_PATH}/dockerized-norlab-images/container-tools && \
    sed "s;alias dn_;${SP}$ dn_;" ./dn_bash_alias.bash | sed "s;='.*;;" | grep -e dn_ \
 )
\033[0m"

echo -e "IDE remote development workflow › to connect to the container internal ssh server:
\033[1;37m
${SP}$ ssh -p ${DN_SSH_SERVER_PORT} ${DN_SSH_SERVER_USER}@$(hostname -I | awk '{print $1}')
${SP}$ sftp -P ${DN_SSH_SERVER_PORT} openssh-$(hostname -I | awk '{print $1}')
${SP}$ scp -P ${DN_SSH_SERVER_PORT} /path/to/source ${DN_SSH_SERVER_USER}@$(hostname -I | awk '{print $1}'):/target/dir/
\033[0m"

# (NICE TO HAVE) ToDo: Add >> procedure for configuring .env file
echo -e "Terminal prompt › The default Dockerized-NorLab prompt require that\033[1;37m Powerline-status\033[0m
or\033[1;37m Powerline10k\033[0m be installed on the host terminal. To change to a minimal prompt,
either set permanently the ENV variable in\033[1;37m docker-compose.project.run.<host-arch>.yaml\033[0m:
${SP}
${SP}services:
${SP}  develop: # the service name
${SP}\033[1;37m    environment:
${SP}      - DN_ACTIVATE_POWERLINE_PROMT=false
\033[0m
or pass the following flag to \033[1;37mdn_attach\033[0m when connecting to a running container:
\033[1;37m
${SP}$ dn_attach --env=\"DN_ACTIVATE_POWERLINE_PROMT=false\" <the-running-container-name>
\033[0m"

echo -e "If you're running a python interpreter in remote development, dont forget to add the python path
pointing to the ROS package:
\033[1;37m
${SP} PYTHONPATH=${PYTHONPATH}
\033[0m
To fetch container environment variables and expose them to the host computer through a mounted
volume \033[1;37m dockerized-norlab-tools/dn_container_env_variable/ \033[0m making them availbale to configure your IDE
(e.g.: PyCharm run configuration using EnvFile plugin https://github.com/Ashald/EnvFile) execute:
\033[1;37m
${SP} $ dn_expose_container_env_variables
\033[0m "
n2st::draw_horizontal_line_across_the_terminal_window '.'

# ====Teardown=====================================================================================
popd >/dev/null || exit 1

