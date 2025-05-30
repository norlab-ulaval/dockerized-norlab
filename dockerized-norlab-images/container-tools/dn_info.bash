#!/bin/bash
#=================================================================================================
# This script is used to display the runtime information of a Docker container. It fetches
# essential environment information, pip package versions, available aliases, service-specific
# info, terminal prompt configuration, and remote python interpreter setup information.
#
# Usage Example:
#   $ source dn_info.bash

# Globals:
#   Read (required):
#     DN_PROJECT_SERVICE       - Name of the Dockerized NorLab service (e.g. 'project-develop')
#     DN_CONTAINER_NAME        - Name of the Docker container
#     DN_PROJECT_USER          - Username responsible for the Docker project
#     DN_PATH                  - Path of the Dockerized NorLab image
#     DN_SSH_SERVER_PORT       - SSH port for the Docker service
#     DN_PROJECT_PATH          - Path of the Docker project
#     DN_DEV_WORKSPACE         - Development workspace within the Docker container
#     DN_PROJECT_GIT_NAME      - Name of the Git repository related to the Docker project
#     DN_PROJECT_GIT_DOMAIN    - Domain of the Git repository related to the Docker project
#     DN_ACTIVATE_POWERLINE_PROMT - Flag indicating whether to activate Powerline prompt in terminal
#
#   Read (optional):
#     ROS_DISTRO               - Version of ROS downloaded in the Docker container
#     ROS_PKG                  - ROS package used in the Docker project
#     ROS_DOMAIN_ID            - ID related to the ROS domain
#     RMW_IMPLEMENTATION       - Implementation details for ROS MiddleWare
#     PYTHONPATH               - The python path
#
# Outputs:
#   Prints container runtime information to stdout.
#
#=================================================================================================
pushd "$(pwd)" >/dev/null || exit 1

# ....Source project shell-scripts dependencies....................................................
# ToDo: validate (ref task NMO-385 sub-task NMO-485 fix: Project related env var overridden issue)
cd /dockerized-norlab/dockerized-norlab-images/container-tools || exit 1
source import_dockerized_norlab_container_tools.bash


function dn::show_container_runtime_information() {

  local DN_PYTHON3_VERSION
  local DN_IMAGE_ARCHITECTURE
  local PKG_VERSION

  # ....Check pre-conditions.......................................................................
  {
    test -n "${DN_PROJECT_SERVICE:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_CONTAINER_NAME:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PATH:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_PATH:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_GIT_NAME:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_GIT_DOMAIN:?'Env variable need to be set and non-empty.'}" ;
  } || exit 1

  if [[ "${DN_PROJECT_SERVICE}" == "project-develop" ]]; then
    test -n "${DN_SSH_SERVER_PORT:?'Env variable need to be set and non-empty.'}" || exit 1
  fi

  # ....Fetch values...............................................................................
  n2st::set_which_python3_version
  DN_PYTHON3_VERSION=${PYTHON3_VERSION?err}

  # (Priority) ToDo: replace `DN_IMAGE_ARCHITECTURE` by `DN_HOST` wich is declare in DN-project
  #    and set in docker compose files
  n2st::set_which_architecture_and_os
  DN_IMAGE_ARCHITECTURE=${IMAGE_ARCH_AND_OS:?err}

  PKG_VERSION=$(pip3 list --format freeze)

  # ====Show runtime in-container information======================================================
  SP="    "
  echo
  n2st::draw_horizontal_line_across_the_terminal_window '.'
  # (NICE TO HAVE) ToDo: refactor  using 'n2st::print_msg' fct
  echo -e "In-container informations:"
  echo -e "\033[1;37m
  ${SP}DN service name:             ${DN_PROJECT_SERVICE}
  ${SP}DN container name:           ${DN_CONTAINER_NAME}
  ${SP}DN user:                     $(whoami)
  ${SP}DN host name:                $(hostname)
  ${SP}DN image architecture:       ${DN_IMAGE_ARCHITECTURE}
  ${SP}DN activate powerline promt: ${DN_ACTIVATE_POWERLINE_PROMT}
  ${SP}
  ${SP}DN target project repo:      https://github.com/${DN_PROJECT_GIT_DOMAIN}/${DN_PROJECT_GIT_NAME}.git
  ${SP}DN project src path:         ${DN_PROJECT_PATH}  \033[0m"
  if [[ -n ${ROS_DISTRO} ]]; then
    echo -e "\033[1;37m
  ${SP}ROS distro:                  ${ROS_DISTRO}
  ${SP}ROS package:                 ${ROS_PKG}
  ${SP}ROS domain id:               ${ROS_DOMAIN_ID}
  ${SP}ROS container workspace:     ${DN_DEV_WORKSPACE}
  ${SP}RMW_IMPLEMENTATION:          ${RMW_IMPLEMENTATION}  \033[0m"
  fi
  echo -e "\033[1;37m
  ${SP}python3 version:             ${DN_PYTHON3_VERSION}
  ${SP}numpy version:               $(echo "${PKG_VERSION}" | grep numpy== | sed 's/numpy==//g')
  ${SP}pyTorch version:             $(echo "${PKG_VERSION}" | grep -w torch | sed 's/torch==//g')
  ${SP}torchvision version:         $(echo "${PKG_VERSION}" | grep -w torchvision | sed 's/torchvision==//g')
  ${SP}numba version:               $(echo "${PKG_VERSION}" | grep numba | sed 's/numba==//g')
  ${SP}LLVMlite version:            $(echo "${PKG_VERSION}" | grep llvmlite | sed 's/llvmlite==//g')
  \033[0m"


  # ....DN wide aliases............................................................................
  echo -e "In-container available alias:
\033[1;37m
$(
cd "${DN_PATH}"/dockerized-norlab-images/container-tools &&
sed "s;alias dn_;${SP}  $ dn_;" ./dn_bash_alias.bash | sed "s;='.*;;" | sed "s;\# dn_.*;;" | grep -e dn_
)
\033[0m"

  # ....Service specific (project-develop) ........................................................
  if [[ "${DN_PROJECT_SERVICE}" == "project-develop" ]]; then
  echo -e "IDE remote development workflow › to connect to the container internal ssh server:
  \033[1;37m
  ${SP}$ ssh -p ${DN_SSH_SERVER_PORT} ${DN_PROJECT_USER}@$(hostname -I | awk '{print $1}')
  ${SP}$ scp -P ${DN_SSH_SERVER_PORT} /path/to/source ${DN_PROJECT_USER}@$(hostname -I | awk '{print $1}'):/target/dir/
  ${SP}$ sftp -P ${DN_SSH_SERVER_PORT} openssh-$(hostname -I | awk '{print $1}')
  \033[0m"
  fi

  # ....Prompt customisation.......................................................................
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

  # ....Remote python interpreter setup info.......................................................
  echo -e "If you're running a python interpreter in remote development mode, dont forget to add the python path
  pointing to the ROS package in your host machine IDE:
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

  return 0
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  MSG_ERROR_FORMAT="\033[1;31m"
  MSG_END_FORMAT="\033[0m"
  echo -e "${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dn::show_container_runtime_information
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
