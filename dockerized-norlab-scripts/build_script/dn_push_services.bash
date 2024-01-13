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

# ....Pre-condition................................................................................

if [[ ! -f  ".env.dockerized-norlab-build-system" ]]; then
  echo -e "\n[\033[1;31mERROR\033[0m] 'dn_build_all.bash' script must be executed from the project root!\n Curent working directory is '$(pwd)'"  1>&2
  exit 1
fi

# ....Load environment variables from file.........................................................
set -o allexport
source .env.dockerized-norlab-build-system || exit 1

# Set PROJECT_GIT_REMOTE_URL
source "${N2ST_PATH:?'Variable not set'}"/.env.project || exit 1

cd "${NBS_PATH:?'Variable not set'}"  || exit 1
source import_norlab_build_system_lib.bash || exit 1

cd "${DN_PATH:?'Variable not set'}" || exit 1
set +o allexport


# ====Begin========================================================================================

set -o allexport
source "${_DOTENV_BUILD_MATRIX_MAIN}"
set +o allexport

# ....execute......................................................................................
export NBS_OVERRIDE_BUILD_MATRIX_MAIN="${_DOTENV_BUILD_MATRIX_MAIN}"

# (NICE TO HAVE) ToDo: refactor to use 'dn_build_all.bash' with NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG and NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY
bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                "${NBS_BUILD_MATRIX_CONFIG:?'Variable not set'}/${NBS_DOTENV_BUILD_MATRIX_ARRAY[*]}" \
                "$@" -- push

# docker compose push relevant flags:
#      --ignore-push-failures   Push what it can and ignores images with push failures
