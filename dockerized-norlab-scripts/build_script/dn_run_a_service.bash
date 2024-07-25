#!/bin/bash
# =================================================================================================
#
# Quickly spawn DN container instance(s) without having to worry about name collision
# i.e. IamNorLab-97168
#
# Usage:
#   $ bash dn_run_a_service.bash '<.env.build_matrix.*>' <service> [<optional command>]
#
# Arguments:
#   - <.env.build_matrix.*>  Dotenv build matrix file
#   - <service>              The service to run
#   - [<optional command>]   Any optional flag for docker compose run <service> [command]
#
# Global e.g.: $ OPTION_FLAG=--build
#
# =================================================================================================
#clear

_DOTENV_BUILD_MATRIX="${1:?' Missing the dotenv build matrix file mandatory argument'}"
shift # Remove argument value

MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"
SCRIP_NAME=dn_run_a_service.bash

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


# ....Execute......................................................................................
n2st::print_formated_script_header "${SCRIP_NAME}" "${MSG_LINE_CHAR_BUILDER_LVL1}"

DN_CONTAINER_NAME="${DN_CONTAINER_NAME:-"IamNorLab"}-${BASHPID:-$$}"

echo
n2st::print_msg "Be advised that ${MSG_DIMMED_FORMAT}${SCRIP_NAME}${MSG_END_FORMAT} bypass the container_name field of the docker compose file, so that you can spin the same service multiple time and all spawned containers will have a unique name, i.e. ${MSG_DIMMED_FORMAT}${DN_CONTAINER_NAME}${MSG_END_FORMAT}"
echo

#OPTION_FLAG=${OPTION_FLAG:-"( --build )"}
OPTION_FLAG=( --build )
#OPTION_FLAG=( --user pycharm-debugger )
#OPTION_FLAG=()
bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                "${NBS_BUILD_MATRIX_CONFIG:?'Variable not set'}/${_DOTENV_BUILD_MATRIX}" \
                --fail-fast \
                -- run "${OPTION_FLAG[@]:-""}" --rm --no-deps --name "${DN_CONTAINER_NAME}" --env "DN_CONTAINER_NAME=${DN_CONTAINER_NAME}" "$@"

n2st::print_formated_script_footer "${SCRIP_NAME}" "${MSG_LINE_CHAR_BUILDER_LVL1}"
