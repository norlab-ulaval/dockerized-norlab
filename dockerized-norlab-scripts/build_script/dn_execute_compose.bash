#!/bin/bash
# =================================================================================================
#
# Build and run a single container based on docker compose docker-compose.yaml
#
# Usage example:
#   $ cd <path/to/dockerized-norlab/root>
#   $ source dockerized-norlab-scripts/build_script/dn_execute_compose.bash
#   $ dn::execute_compose docker-compose.project.yaml -- build --push dependencies-core
#   $ DOCKER_EXIT_CODE=$?
#
# Signature:
#   $ dn::execute_compose <docker-compose.yaml> [<optional flag>] [-- <any docker cmd+arg>]
#
# Arguments:
#   see function print_help_in_terminal or execute the script with the --help flag
#
# Note:
#   Dont use "set -e" in this script as it will affect the build system policy, use the --fail-fast flag instead
#
# =================================================================================================
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"


function dn::execute_compose() {
  # ....Positional argument........................................................................
  local COMPOSE_FILE="${1:?'Missing the docker-compose.yaml file mandatory argument'}"
  shift # Remove argument value

  # ....Default....................................................................................
  local DOCKER_MANAGEMENT_COMMAND=(compose)
  declare -a DOCKER_COMPOSE_CMD_ARGS  # eg: 'build --no-cache --push' or 'up --build --force-recreate'
  _CI_TEST=false

  # (CRITICAL) ToDo: refactor env var setting related to docker compose cmd. Return an error if not set explicitly via flag or use new mechanism to register build arg (NMO-396)
  REPOSITORY_VERSION='latest'
  BASE_IMAGE='dustynv/ros'
  OS_NAME='l4t'
  TAG_PACKAGE='foxy-pytorch-l4t'
  TAG_VERSION='r35.2.1'

  # ....Pre-condition..............................................................................

  if [[ ! -f  ".env.dockerized-norlab-build-system" ]]; then
    echo -e "\n[${MSG_ERROR_FORMAT}DN ERROR${MSG_END_FORMAT}] 'dn::execute_compose' function must be executed from the project root!\n Curent working directory is '$(pwd)'" 1>&2
    exit 1
  fi


  if [[ ! -f  "${COMPOSE_FILE}" ]]; then
    echo -e "\n[${MSG_ERROR_FORMAT}DN ERROR${MSG_END_FORMAT}] 'dn::execute_compose' can't find the docker-compose.yaml file '${COMPOSE_FILE}' at $(pwd)"  1>&2
    exit 1
  fi

  # ....Load environment variables from file.......................................................
  set -o allexport
  source .env.dockerized-norlab-build-system
  source "${N2ST_PATH:?'Variable not set'}"/.env.project
  set +o allexport

  set -o allexport
  # (StandBy) iceboxed: move at the last possible execution step
  source .env.dockerized-norlab-project
  set +o allexport

  # ....Helper function............................................................................
  ## import shell functions from norlab-shell-script-tools utilities library

# ToDo: on task end >> delete next bloc ↓↓
#  local TMP_CWD_EC=$(pwd)
#  cd "$N2ST_PATH"/src/function_library
#  source ./prompt_utilities.bash
#  source ./docker_utilities.bash
#  source ./general_utilities.bash
#  source ./teamcity_utilities.bash
#  source ./terminal_splash.bash
#  cd "$TMP_CWD_EC"
#

  function print_help_in_terminal() {
    echo -e "\n
  \$ dn::execute_compose <docker-compose.yaml> [<optional flag>] [-- <any docker cmd+arg>]
    \033[1m
      <optional argument>:\033[0m
        -h, --help                              Get help
        --dockerized-norlab-version v1.3.1      The dockerized-norlab release tag (default to main branch latest)
        --base-image                            The base image name (default to 'dustynv/ros')
        --os-name                               The name os the OS (default to 'l4t')
        --tag-package                           The package name portion of the tag (default to 'foxy-pytorch-l4t')
        --tag-version r35.2.1                   Operating system version, see .env.build_matrix for supported version
                                                  (default to 'r35.2.1')
                                                Note: L4T container tags (e.g. r35.2.1) should match the L4T version
                                                on the Jetson otherwize cuda driver won't be accessible
                                                (source https://github.com/dusty-nv/jetson-containers#pre-built-container-images )
        --buildx-bake                           Use 'docker buildx bake <cmd>' instead of 'docker compose <cmd>'
        --docker-debug-logs                     Set Docker builder log output for debug (i.e.BUILDKIT_PROGRESS=plain)
        --fail-fast                             Exit script at first encountered error
        --ci-test-force-runing-docker-cmd

    \033[1m
      [-- <any docker cmd+arg>]\033[0m                 Any argument passed after '--' will be passed to docker compose as docker
                                                command and arguments (default to '${DOCKER_COMPOSE_CMD_ARGS[*]}').
                                                Note: passing script flag via docker --build-arg can be tricky,
                                                      pass them in the docker-compose.yaml if you experience problem.
  "
  }

  # ....TeamCity service message logic.............................................................
  if [[ ${TEAMCITY_VERSION} ]]; then
    export IS_TEAMCITY_RUN=true
    TC_VERSION="TEAMCITY_VERSION=${TEAMCITY_VERSION}"
  else
    export IS_TEAMCITY_RUN=false
  fi
  #printenv
  print_msg "IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN} ${TC_VERSION}"

  # ====Begin======================================================================================
  SHOW_SPLASH_EC="${SHOW_SPLASH_EC:-true}"

  if [[ "${SHOW_SPLASH_EC}" == 'true' ]]; then
    norlab_splash "${NBS_SPLASH_NAME}" "${PROJECT_GIT_REMOTE_URL}"
  fi

  print_formated_script_header 'dn_execute_compose.bash' "${MSG_LINE_CHAR_BUILDER_LVL2}"


  # ....Script command line flags..................................................................
  while [ $# -gt 0 ]; do

    case $1 in
    --dockerized-norlab-version)
      REPOSITORY_VERSION="${2}"
      shift # Remove argument (--dockerized-norlab-version)
      shift # Remove argument value
      ;;
    --base-image)
      BASE_IMAGE="${2}"
      shift # Remove argument (--base-image)
      shift # Remove argument value
      ;;
    --os-name)
      OS_NAME="${2}"
      shift # Remove argument (--os-name)
      shift # Remove argument value
      ;;
    --tag-package)
      TAG_PACKAGE="${2}"
      shift # Remove argument (--tag-package)
      shift # Remove argument value
      ;;
    --tag-version)
      TAG_VERSION="${2}"
      shift # Remove argument (--tag-version)
      shift # Remove argument value
      ;;
    --docker-debug-logs)
  #    set -v
  #    set -x
      export BUILDKIT_PROGRESS=plain
      shift # Remove argument (--docker-debug-logs)
      ;;
    --buildx-bake)
      print_msg_warning "Be advise, the DN --buildx-bake flag is still in developemenmt. Use at your own risk"
      DOCKER_MANAGEMENT_COMMAND=(buildx bake)
      shift # Remove argument (--buildx-bake)
      ;;
    --fail-fast)
      set -e
      shift # Remove argument (--fail-fast)
      ;;
    --ci-test-force-runing-docker-cmd)
      _CI_TEST=true
      shift # Remove argument (--ci-test-force-runing-docker-cmd)
      ;;
    -h | --help)
      print_help_in_terminal
      exit
      ;;
    --) # no more option
      shift
      DOCKER_COMPOSE_CMD_ARGS=( $@ )
      break
      ;;
    *) # Default case
      break
      ;;
    esac

  done

  # ...............................................................................................
  # Note: REPOSITORY_VERSION will be used to fetch the repo at release tag (ref task NMO-252)
  export REPOSITORY_VERSION="${REPOSITORY_VERSION}"
  export DEPENDENCIES_BASE_IMAGE="${BASE_IMAGE}"
  export TAG_VERSION="${TAG_VERSION}"

  export PROJECT_TAG="${OS_NAME}-${TAG_VERSION}"

  export DEPENDENCIES_BASE_IMAGE_TAG="${TAG_PACKAGE}-${TAG_VERSION}"
  export DN_IMAGE_TAG="DN-${REPOSITORY_VERSION}-${DEPENDENCIES_BASE_IMAGE_TAG}"

  print_msg "Environment variables set for compose:\n
  ${MSG_DIMMED_FORMAT}    REPOSITORY_VERSION=${REPOSITORY_VERSION} ${MSG_END_FORMAT}
  ${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE=${DEPENDENCIES_BASE_IMAGE} ${MSG_END_FORMAT}
  ${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE_TAG=${DEPENDENCIES_BASE_IMAGE_TAG} ${MSG_END_FORMAT}
  "

  print_msg "Executing docker compose command on ${MSG_DIMMED_FORMAT}${COMPOSE_FILE}${MSG_END_FORMAT} with command ${MSG_DIMMED_FORMAT}${DOCKER_COMPOSE_CMD_ARGS[*]}${MSG_END_FORMAT}"
  print_msg "Image tag ${MSG_DIMMED_FORMAT}${DN_IMAGE_TAG}${MSG_END_FORMAT}"
  #${MSG_DIMMED_FORMAT}$(printenv | grep -i -e LPM_ -e DEPENDENCIES_BASE_IMAGE -e BUILDKIT)${MSG_END_FORMAT}

  # ToDo: modularity feat › refactor clause as a callback pre bash script and add `source <callback-pre.bash>` with auto discovery logic eg: path specified in the .env.build_matrix

  # =================================================================================================
  # Pre docker command execution callback
  #
  # Usage:
  #   $ callback_execute_compose_pre()
  #
  # Globals:
  #   Read COMPOSE_FILE
  #   Read DEPENDENCIES_BASE_IMAGE
  #   Read DEPENDENCIES_BASE_IMAGE_TAG
  #
  # =================================================================================================
  function callback_execute_compose_pre() {
    if [[ ! -f ${COMPOSE_FILE} ]]; then
      print_msg_error_and_exit "docker-compose file ${COMPOSE_FILE} is unreachable"
    fi

    if [[ "${COMPOSE_FILE}" == "dockerized-norlab-images/core-images/dependencies/docker-compose.dn-dependencies.build.yaml" ]]; then
      # ex: dustynv/ros:foxy-pytorch-l4t-r35.2.1

      # shellcheck disable=SC2046
      docker pull "${DEPENDENCIES_BASE_IMAGE}:${DEPENDENCIES_BASE_IMAGE_TAG}" \
      && export $(docker inspect --format='{{range .Config.Env}}{{println .}}{{end}}' "${DEPENDENCIES_BASE_IMAGE}:${DEPENDENCIES_BASE_IMAGE_TAG}" \
        | grep \
          -e CUDA_HOME= \
          -e NVIDIA_ \
          -e LD_LIBRARY_PATH= \
          -e PATH= \
          -e ROS_ \
          -e RMW_IMPLEMENTATION= \
          -e LD_PRELOAD= \
          -e OPENBLAS_CORETYPE= \
        | sed 's;^CUDA_HOME;BASE_IMG_ENV_CUDA_HOME;' \
        | sed 's;^NVIDIA_;BASE_IMG_ENV_NVIDIA_;' \
        | sed 's;^PATH;BASE_IMG_ENV_PATH;' \
        | sed 's;^LD_LIBRARY_PATH;BASE_IMG_ENV_LD_LIBRARY_PATH;' \
        | sed 's;^ROS_;BASE_IMG_ENV_ROS_;' \
        | sed 's;^RMW_IMPLEMENTATION;BASE_IMG_ENV_RMW_IMPLEMENTATION;' \
        | sed 's;^LD_PRELOAD;BASE_IMG_ENV_LD_PRELOAD;' \
        | sed 's;^OPENBLAS_CORETYPE;BASE_IMG_ENV_OPENBLAS_CORETYPE;' \
       )

      print_msg "Passing the following environment variable from ${MSG_DIMMED_FORMAT}${DEPENDENCIES_BASE_IMAGE}:${DEPENDENCIES_BASE_IMAGE_TAG}${MSG_END_FORMAT} to ${MSG_DIMMED_FORMAT}${DN_HUB:?err}/dn-dependencies-core:${DN_IMAGE_TAG}${MSG_END_FORMAT}:
        ${MSG_DIMMED_FORMAT}\n$(printenv | grep -e BASE_IMG_ENV_ | sed 's;BASE_IMG_ENV_;    ;')
        ${MSG_END_FORMAT}"
    fi
  }

  callback_execute_compose_pre

  n2st::show_and_execute_docker "${DOCKER_MANAGEMENT_COMMAND[*]} -f ${COMPOSE_FILE} ${DOCKER_COMPOSE_CMD_ARGS[*]}" "$_CI_TEST"

  # ToDo: modularity feat › add `source <callback-post.bash>` with auto discovery logic eg: path specified in the .env.build_matrix

  print_msg "Environment variables used by compose:\n
  ${MSG_DIMMED_FORMAT}    REPOSITORY_VERSION=${REPOSITORY_VERSION} ${MSG_END_FORMAT}
  ${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE=${DEPENDENCIES_BASE_IMAGE} ${MSG_END_FORMAT}
  ${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE_TAG=${DEPENDENCIES_BASE_IMAGE_TAG} ${MSG_END_FORMAT}"

  print_formated_script_footer 'dn_execute_compose.bash' "${MSG_LINE_CHAR_BUILDER_LVL2}"

  # ====Teardown===================================================================================
  cd "${TMP_CWD_EC}"

  return "${DOCKER_EXIT_CODE:?"variable was not set by n2st::show_and_execute_docker"}"
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  echo -e "\n[${MSG_ERROR_FORMAT}DN ERROR${MSG_END_FORMAT}] This script must be sourced  i.e.: $ source dn_execute_compose.bash" 1>&2
  exit 1

else
  # This script is being sourced, ie: __name__="__source__"

  if [[ "${NBS_IMPORTED}" != "true" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} You need to execute ${MSG_DIMMED_FORMAT}import_norlab_build_system_lib.bash${MSG_END_FORMAT} before sourcing ${MSG_DIMMED_FORMAT}dn_execute_compose.bash${MSG_END_FORMAT}." 1>&2
    exit 1
  else
    # NBS was imported prior to the script execution
    :
  fi

fi
