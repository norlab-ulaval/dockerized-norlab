#!/bin/bash
#
# Build and run a single container based on docker compose docker-compose.dn-dependencies.build.yaml
#
# Usage:
#   $ bash dn_execute_compose.bash <docker-compose.yaml> [<optional flag>] [-- <any docker cmd+arg>]
#
#   $ bash dn_execute_compose.bash <docker-compose.yaml> -- build --push dependencies-core
#
# Arguments:
#   see function print_help_in_terminal or execute the script with the --help flag
#
# Note:
#   Dont use "set -e" in this script as it will affect the build system policy, use the --fail-fast flag instead
#

# ....Default.......................................................................................
DOCKERIZED_NORLAB_VERSION='latest'
BASE_IMAGE='dustynv/ros'
TAG_PACKAGE='foxy-pytorch-l4t'
TAG_VERSION='r35.2.1'
#LPM_JOB_ID='0'
DOCKER_COMPOSE_CMD_ARGS='build'  # eg: 'build --no-cache --push' or 'up --build --force-recreate'
CI_TEST=false

PATH_TO_COMPOSE_FILE_DIR='dockerized-norlab-images/compose-matrix'
EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE="${1:?'Missing the docker-compose.yaml file mandatory argument'}"
shift # Remove argument value


# ....Pre-condition.................................................................................
if [[ ! -f  ".env.dockerized-norlab" ]]; then
  echo -e "\n[\033[1;31mERROR\033[0m] 'dn_execute_compose.bash' script must be sourced from the project root!\n Curent working directory is '$(pwd)'"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi


if [[ ! -f  "${PATH_TO_COMPOSE_FILE_DIR}/${EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE}" ]]; then
  echo -e "\n[\033[1;31mERROR\033[0m] 'dn_execute_compose.bash' can't find the docker-compose.yaml file '${EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE}'"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ....Load environment variables from file....................................................................
set -o allexport
source .env.dockerized-norlab
set +o allexport

set -o allexport
source ./utilities/norlab-shell-script-tools/.env.project
set +o allexport

# ....Helper function...............................................................................
## import shell functions from norlab-shell-script-tools utilities library

TMP_CWD=$(pwd)
cd ./utilities/norlab-shell-script-tools/src/function_library
source ./prompt_utilities.bash
source ./docker_utilities.bash
source ./general_utilities.bash
source ./teamcity_utilities.bash
source ./terminal_splash.bash
cd "$TMP_CWD"


function print_help_in_terminal() {
  echo -e "\n
\$ ${0} <docker-compose.yaml> [<optional flag>] [-- <any docker cmd+arg>]
  \033[1m
    <optional argument>:\033[0m
      -h, --help                              Get help
      --dockerized-norlab-version v1.3.1      The dockerized-norlab release tag (default to main branch latest)
      --base-image                            The base image name (default to 'dustynv/ros')
      --tag-package                           The package name portion of the tag (default to 'foxy-pytorch-l4t')
      --tag-version r35.2.1                   Operating system version, see .env.build_matrix for supported version
                                                (default to 'r35.2.1')
                                              Note: L4T container tags (e.g. r35.2.1) should match the L4T version
                                              on the Jetson otherwize cuda driver won't be accessible
                                              (source https://github.com/dusty-nv/jetson-containers#pre-built-container-images )
      --docker-debug-logs                     Set Docker builder log output for debug (i.e.BUILDKIT_PROGRESS=plain)
      --fail-fast                             Exit script at first encountered error
      --ci-test-force-runing-docker-cmd

  \033[1m
    [-- <any docker cmd+arg>]\033[0m                 Any argument passed after '--' will be passed to docker compose as docker
                                              command and arguments (default to '${DOCKER_COMPOSE_CMD_ARGS}').
                                              Note: passing script flag via docker --build-arg can be tricky,
                                                    pass them in the docker-compose.yaml if you experience problem.
"
}

# ....TeamCity service message logic.................................................................
if [[ ${TEAMCITY_VERSION} ]]; then
  export IS_TEAMCITY_RUN=true
  TC_VERSION="TEAMCITY_VERSION=${TEAMCITY_VERSION}"
else
  export IS_TEAMCITY_RUN=false
fi
#printenv
print_msg "IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN} ${TC_VERSION}"

# ====Begin=========================================================================================
SHOW_SPLASH_EC="${SHOW_SPLASH_EC:-true}"

if [[ "${SHOW_SPLASH_EC}" == 'true' ]]; then
  norlab_splash "${DN_SPLASH_NAME}" "${PROJECT_GIT_REMOTE_URL}"
fi

print_formated_script_header 'dn_execute_compose.bash' "${MSG_LINE_CHAR_BUILDER_LVL2}"


# ....Script command line flags.....................................................................
while [ $# -gt 0 ]; do

  case $1 in
  --dockerized-norlab-version)
    DOCKERIZED_NORLAB_VERSION="${2}"
    shift # Remove argument (--dockerized-norlab-version)
    shift # Remove argument value
    ;;
  --base-image)
    BASE_IMAGE="${2}"
    shift # Remove argument (--base-image)
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
  --fail-fast)
    set -e
    shift # Remove argument (--fail-fast)
    ;;
  --ci-test-force-runing-docker-cmd)
    CI_TEST=true
    shift # Remove argument (--fail-fast)
    ;;
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  --) # no more option
    shift
    DOCKER_COMPOSE_CMD_ARGS="$*"
    break
    ;;
  *) # Default case
    break
    ;;
  esac

done

# ...................................................................................................
# Note: DOCKERIZED_NORLAB_VERSION will be used to fetch the repo at release tag (ref task NMO-252)
export DOCKERIZED_NORLAB_VERSION="${DOCKERIZED_NORLAB_VERSION}"
export DEPENDENCIES_BASE_IMAGE="${BASE_IMAGE}"
export TAG_VERSION="${TAG_VERSION}"
export DEPENDENCIES_BASE_IMAGE_TAG="${TAG_PACKAGE}-${TAG_VERSION}"

export DN_IMAGE_TAG="DN${DOCKERIZED_NORLAB_VERSION}-JC-${DEPENDENCIES_BASE_IMAGE_TAG}"

print_msg "Environment variables set for compose:\n
${MSG_DIMMED_FORMAT}    DOCKERIZED_NORLAB_VERSION=${DOCKERIZED_NORLAB_VERSION} ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE=${DEPENDENCIES_BASE_IMAGE} ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE_TAG=${DEPENDENCIES_BASE_IMAGE_TAG} ${MSG_END_FORMAT}
"

print_msg "Executing docker compose command on ${MSG_DIMMED_FORMAT}docker-compose.dockerized-norlab.build.yaml${MSG_END_FORMAT} with command ${MSG_DIMMED_FORMAT}${DOCKER_COMPOSE_CMD_ARGS}${MSG_END_FORMAT}"
print_msg "Image tag ${MSG_DIMMED_FORMAT}${DN_IMAGE_TAG}${MSG_END_FORMAT}"
#${MSG_DIMMED_FORMAT}$(printenv | grep -i -e LPM_ -e DEPENDENCIES_BASE_IMAGE -e BUILDKIT)${MSG_END_FORMAT}

## docker compose [-f <theComposeFile> ...] [options] [COMMAND] [ARGS...]
## docker compose build [OPTIONS] [SERVICE...]
## docker compose run [OPTIONS] SERVICE [COMMAND] [ARGS...]

show_and_execute_docker "compose -f ${PATH_TO_COMPOSE_FILE_DIR}/${EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE} ${DOCKER_COMPOSE_CMD_ARGS}" "$CI_TEST"

print_msg "Environment variables used by compose:\n
${MSG_DIMMED_FORMAT}    DOCKERIZED_NORLAB_VERSION=${DOCKERIZED_NORLAB_VERSION} ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE=${DEPENDENCIES_BASE_IMAGE} ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE_TAG=${DEPENDENCIES_BASE_IMAGE_TAG} ${MSG_END_FORMAT}"

print_formated_script_footer 'dn_execute_compose.bash' "${MSG_LINE_CHAR_BUILDER_LVL2}"
# ====Teardown======================================================================================
cd "${TMP_CWD}"
