#!/bin/bash
# =================================================================================================
#
# Convenient script for building all images specified in in a dotenv build matrix file
#
# Usage:
#   $ bash dn_build_over_single_build_matrix.bash '<.env.build_matrix.main>' [<optional-flag>]
#
# Arguments:
#   - <.env.build_matrix.main>      A main dotenv build matrix file
#   - [<optional-flag>]             Any optional flag from 'dn_execute_compose_over_build_matrix.bash'
#
# Global
#   - Read NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG    Use to quickly add docker flag at runtime
#                               e.g.: $ NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG="build --push myService" && bash dn_build_over_single_build_matrix.bash
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
# ....setup........................................................................................
# Note: 'NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG' is set via commandline for convenience
DOCKER_COMMAND_W_FLAGS="${NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG:-"build"}"


set -o allexport
source "${_DOTENV_BUILD_MATRIX_MAIN}"
set +o allexport

# Note: NBS_DOTENV_BUILD_MATRIX_ARRAY is specify in .env.build_matrix.main
PARAMS=( "${NBS_BUILD_MATRIX_CONFIG:?'Variable not set'}/${NBS_DOTENV_BUILD_MATRIX_ARRAY[*]}" "$@" -- "$DOCKER_COMMAND_W_FLAGS" )

# ....execute......................................................................................
export NBS_OVERRIDE_BUILD_MATRIX_MAIN="${_DOTENV_BUILD_MATRIX_MAIN}"

bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash "${PARAMS[@]}"

