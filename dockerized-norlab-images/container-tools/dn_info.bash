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

  # ToDo: NMO-669 refactor: replace DN_IMAGE_ARCHITECTURE by DN_HOST which is declare in DN-project
  n2st::set_which_architecture_and_os
  DN_IMAGE_ARCHITECTURE=${IMAGE_ARCH_AND_OS:?err}

  PKG_VERSION=$(pip3 list --format freeze)

  # ====Show runtime in-container information======================================================
  local _sp="    "
  echo
  local line_style="─"
  n2st::draw_horizontal_line_across_the_terminal_window "${line_style}" "${MSG_DIMMED_FORMAT}"
  # (NICE TO HAVE) ToDo: refactor  using 'n2st::print_msg' fct
  echo -e "${MSG_EMPH_FORMAT}In-container informations${MSG_END_FORMAT} ›

${_sp}DN service name:             ${DN_PROJECT_SERVICE}
${_sp}DN container name:           ${DN_CONTAINER_NAME}
${_sp}DN user:                     $(whoami)
${_sp}DN host name:                $(hostname)
${_sp}DN image architecture:       ${DN_IMAGE_ARCHITECTURE}
${_sp}DN activate powerline promt: ${DN_ACTIVATE_POWERLINE_PROMT}
${_sp}
${_sp}DN target project repo:      https://github.com/${DN_PROJECT_GIT_DOMAIN}/${DN_PROJECT_GIT_NAME}.git
${_sp}DN project src path:         ${DN_PROJECT_PATH}"
  if [[ -n ${ROS_DISTRO} ]]; then
    echo -e "
${_sp}ROS distro:                  ${ROS_DISTRO}
${_sp}ROS package:                 ${ROS_PKG}
${_sp}ROS domain id:               ${ROS_DOMAIN_ID}
${_sp}ROS container workspace:     ${DN_DEV_WORKSPACE}
${_sp}RMW_IMPLEMENTATION:          ${RMW_IMPLEMENTATION}"
  fi
  echo -e "
${_sp}python3 version:             ${DN_PYTHON3_VERSION}
${_sp}numpy version:               $(echo "${PKG_VERSION}" | grep numpy== | sed 's/numpy==//g')
${_sp}pyTorch version:             $(echo "${PKG_VERSION}" | grep -w torch | sed 's/torch==//g')
${_sp}numba version:               $(echo "${PKG_VERSION}" | grep numba | sed 's/numba==//g')
${_sp}LLVMlite version:            $(echo "${PKG_VERSION}" | grep llvmlite | sed 's/llvmlite==//g')"

#  ${_sp}torchvision version:         $(echo "${PKG_VERSION}" | grep -w torchvision | sed 's/torchvision==//g')

  # ....Prompt customisation.......................................................................
  # (NICE TO HAVE) ToDo: Add >> procedure for configuring .env file
  echo -e "${MSG_EMPH_FORMAT}Terminal prompt configuration${MSG_END_FORMAT} ›
The default Dockerized-NorLab prompt require that\033[1;37m Powerline-status\033[0m or\033[1;37m Powerline10k\033[0m
be installed on the host terminal. To change to a minimal prompt, either set permanently
the environment variable in \033[1;37mDN_ACTIVATE_POWERLINE_PROMT=false\033[0m in .dockerized_norlab/configuration/.env.dna
or pass the following flag to \033[1;37mdna up\033[0m or \033[1;37mdna exec\033[0m when connecting to a running container:
\033[1;37m
${_sp}$ dna [up|exec] --env=\"DN_ACTIVATE_POWERLINE_PROMT=false\" -- bash
\033[0m"

  # ....Service specific (project-develop) ........................................................
  if [[ "${DN_PROJECT_SERVICE}" == "project-develop" ]]; then
  echo -e "${MSG_EMPH_FORMAT}Remote development workflow${MSG_END_FORMAT} ›
To connect to the container internal ssh server:

Case › Interactive and non-interactive shell (regular bash shell)
${_sp}${MSG_EMPH_FORMAT}$ ssh -p ${DN_SSH_SERVER_PORT} ${DN_PROJECT_USER}@$(hostname -I | awk '{print $1}')${MSG_END_FORMAT}

Case › ROS2 pre-sourced non-interactive shell (shell with ros2 support tailormade for ssh python interpreter)
${_sp}${MSG_EMPH_FORMAT}$ ssh -p ${DN_SSH_SERVER_PORT} ${DN_SSH_SERVER_USER}@$(hostname -I | awk '{print $1}')${MSG_END_FORMAT}

Case › Copy file from local host to remote host
${_sp}${MSG_EMPH_FORMAT}$ scp -P ${DN_SSH_SERVER_PORT} /path/to/source ${DN_PROJECT_USER}@$(hostname -I | awk '{print $1}'):/target/dir/${MSG_END_FORMAT}
"

  # ....Remote python interpreter setup info.......................................................
  echo -e "${MSG_EMPH_FORMAT}Remote development workflow with ROS2 support${MSG_END_FORMAT} ›
If you're running a python interpreter in remote development mode and you want ROS2 path and python path mirorred in your local host IDE

${_sp}\033[1;37mOption 1\033[0m:
${_sp}Configure a remote ssh python interpreter using \033[1;37m${DN_SSH_SERVER_USER}@$(hostname -I | awk '{print $1}'):${DN_SSH_SERVER_PORT}\033[0m
${_sp}Its tailormade for that purposes.
${_sp}
${_sp}\033[1;37mOption 2\033[0m:
${_sp}Fetch the container environment variables and expose them to the host computer through the mounted
${_sp}volume \033[1;37m dockerized-norlab-tools/dn_container_env_variable/ \033[0m making them available
${_sp}To configure your ide. Execute the following:
${_sp}
${_sp}\033[1;37m$ dn::source_ros2 && dn-expose-container-env-variables\033[0m
${_sp}or
${_sp}\033[1;37m$ dn::source_ros2_underlay_only && dn-expose-container-env-variables\033[0m
${_sp}
${_sp}Recommend using using EnvFile plugin https://github.com/Ashald/EnvFile for that purposes.
${_sp}
${_sp}\033[1;37mOption 3\033[0m:
${_sp}Add the following python path manualy to your host machine IDE python interpreter configuration:
${_sp}${MSG_DIMMED_FORMAT}
${_sp}PYTHONPATH=$(dn::source_ros2 >/dev/null && echo "${PYTHONPATH}")
${_sp}${MSG_END_FORMAT}
"
  fi

  # ....DN wide aliases............................................................................
  echo
  echo -e "${MSG_EMPH_FORMAT}In-container available alias${MSG_END_FORMAT} ›\n
$(
cd "${DN_PATH}"/dockerized-norlab-images/container-tools &&
sed "s;alias dn-;${_sp}$ dn-;" ./dn_bash_alias.bash | sed "s;='.*;;" | sed "s;\# dn-.*;;" | grep -e dn-
)"


  n2st::draw_horizontal_line_across_the_terminal_window "${line_style}" "${MSG_DIMMED_FORMAT}"

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
