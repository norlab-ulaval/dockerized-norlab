#!/bin/bash

# Note: This is a dev utility script implemented at project early stage.
#       It's still usefull for debug purposes ... but it need to be updated and refactored heavily.
# (CRITICAL) ToDo: Refactor based on up_and_attach.bash from DN-project

# =================================================================================================
#
# Convenient script for spinning a specific service from the docker compose
# file define in a .env.build_matrix
#
# Usage:
#   $ bash dn_up_and_attach_to.bash '<.env.build_matrix.*>' <theService>
#
# Arguments:
#   - <theService>              The service to attach once all are up
#
# =================================================================================================
clear

_DOTENV_BUILD_MATRIX="${1:?' Missing the dotenv build matrix file mandatory argument'}"
THE_SERVICE="${2:?' Missing the docker compose service to run mandatory argument'}"

MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"
SCRIP_NAME=dn_up_and_attach_to.bash

if [[ $(basename "$(pwd)") = build_script ]]; then
  cd ../..
elif [[ $(basename "$(pwd)") = dockerized-norlab-scripts ]]; then
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

DOCKER_UP_FLAGS=()
DOCKER_UP_FLAGS+=( '--build' '--detach' '--wait' '--no-deps' )

bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
  "${NBS_BUILD_MATRIX_CONFIG:?'Variable not set'}/${_DOTENV_BUILD_MATRIX}" \
  --fail-fast -- up "${DOCKER_UP_FLAGS[@]}" "${THE_SERVICE}"

#CN=$(grep -A3 'project-develop:' ${THE_COMPOSE_FILE} | tail -n1); CN=${CN//*container_name: /}; echo "$CN"
#echo "${CN}"

if [[ -n $TEAMCITY_VERSION ]]; then
  # (NICE TO HAVE) ToDo: implement >> fetch container name from an .env file
  echo -e "${DS_MSG_EMPH_FORMAT}The container is running inside a TeamCity agent >> keep container detached${DS_MSG_END_FORMAT}"
else

  bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
    "${NBS_BUILD_MATRIX_CONFIG:?'Variable not set'}/${_DOTENV_BUILD_MATRIX}" \
    --fail-fast \
    -- exec "${THE_SERVICE}" bash
fi

n2st::print_formated_script_footer "${SCRIP_NAME}" "${MSG_LINE_CHAR_BUILDER_LVL1}"
