#!/bin/bash
# =================================================================================================
#
# Convenient script for pushing all images specified in a 'docker-compose.<project-name>.build.yaml'
#
# Usage:
#   $ bash dn_push_services.bash '<.env.build_matrix.*>' [<optional flag>]
#
# Arguments:
#   - <.env.build_matrix.*>  Dotenv build matrix file
#   - [<optional flag>]   Any optional flag from 'dn_execute_compose_over_build_matrix.bash'
#
# =================================================================================================
clear

# The main .env.build_matrix to load
#
_DOTENV_BUILD_MATRIX_MAIN="${1:?' Missing the dotenv build matrix file mandatory argument'}"
shift # Remove argument value


MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"
SCRIP_NAME=dn_push_services.bash

if [[ $( basename "$(pwd)" ) = build_script ]]; then
    cd ../..
elif [[ $( basename "$(pwd)" ) = dockerized-norlab-scripts ]]; then
    cd ..
fi

# ....Pre-condition................................................................................
if [[ ! -f ".env.dockerized-norlab-build-system" ]]; then
  echo -e "\n[${MSG_ERROR_FORMAT}DN ERROR${MSG_END_FORMAT}] '${SCRIP_NAME}' script must be executed from the project root!\n Curent working directory is '$(pwd)'" 1>&2
  exit 1
fi

set -o allexport
source .env.dockerized-norlab-build-system
set +o allexport


# ....Helper function..............................................................................
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then # This script is being run, ie: __name__="__main__"
  cd "${DN_PATH:?err}"
  source import_dockerized_norlab_tools.bash || exit 1
  cd "${DN_PATH}"
else # This script is being sourced, ie: __name__="__source__"
  if [[ ${_CI_TEST} != true ]]; then
    echo -e "\n[${MSG_ERROR_FORMAT}DN ERROR${MSG_END_FORMAT}] Execute this script in a subshell i.e.: $ bash ${SCRIP_NAME}" 1>&2
    exit 1
  else
    if [[ "${NBS_IMPORTED}" != "true" ]]; then
      echo -e "\n${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} You need to execute ${MSG_DIMMED_FORMAT}import_dockerized_norlab_tools.bash${MSG_END_FORMAT} before sourcing ${MSG_DIMMED_FORMAT}${SCRIP_NAME}${MSG_END_FORMAT} otherwise run it with bash." 1>&2
      exit 1
    else # NBS was imported prior to the script execution
      :
    fi
  fi
fi


# ====Begin========================================================================================

set -o allexport
source "${_DOTENV_BUILD_MATRIX_MAIN}"
set +o allexport

# ....execute......................................................................................
n2st::print_formated_script_header "${SCRIP_NAME}" "${MSG_LINE_CHAR_BUILDER_LVL1}"
export NBS_OVERRIDE_BUILD_MATRIX_MAIN="${_DOTENV_BUILD_MATRIX_MAIN}"

# (NICE TO HAVE) ToDo: refactor to use 'dn_build_all.bash' with NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG and NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY
bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                "${NBS_BUILD_MATRIX_CONFIG:?'Variable not set'}/${NBS_DOTENV_BUILD_MATRIX_ARRAY[*]}" \
                "$@" -- push

# docker compose push relevant flags:
#      --ignore-push-failures   Push what it can and ignores images with push failures

n2st::print_formated_script_footer "${SCRIP_NAME}" "${MSG_LINE_CHAR_BUILDER_LVL1}"
