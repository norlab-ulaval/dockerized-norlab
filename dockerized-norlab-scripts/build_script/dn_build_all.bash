#!/bin/bash
# =================================================================================================
#
# Convenient script for building all images by crawling over all ".env.build_matrix.dn-*" file
#
# Usage:

#   $ bash dn_build_all.bash [<optional flag>]
#
# Arguments:
#   - [<optional flag>]   Any optional flag from 'dn_execute_compose_over_build_matrix.bash'
#
# Global
#   - Read NBS_OVERRIDE_BUILD_MATRIX_MAIN          Use to quickly change the .env.build_matrix.main file
#   - Read NBS_OVERRIDE_ADD_DOCKER_FLAG            Use to quickly add docker flag at runtime
#             e.g.: $ NBS_OVERRIDE_ADD_DOCKER_FLAG=--push --dry-run && source dn_build_all.bash
#   - Read NBS_OVERRIDE_DOCKER_CMD         Use to quickly overide the docker command
#             e.g.: $ NBS_OVERRIDE_DOCKER_CMD=push
#   - Read NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY  Use to quickly overide the build matrix list
#             e.g.: $ NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY=( '.env.build_matrix.dev.dn' ) && source dn_build_all.bash
#   - Read STR_BUILD_MATRIX_SERVICES_AND_TAGS from build_all.log
#
# =================================================================================================
clear

# ....Pre-condition................................................................................

if [[ ! -f  ".env.norlab-build-system" ]]; then
  echo -e "\n[\033[1;31mERROR\033[0m] 'dn_build_all.bash' script must be executed from the project root!\n Curent working directory is '$(pwd)'"  1>&2
  exit 1
fi

# ....Load environment variables from file.........................................................

#
# The main .env.build_matrix to load
#
NBS_BUILD_MATRIX_MAIN=${NBS_OVERRIDE_BUILD_MATRIX_MAIN:-".env.build_matrix.main"}

set -o allexport
source .env.norlab-build-system
source "$NBS_BUILD_MATRIX_MAIN"

# Set PROJECT_GIT_REMOTE_URL
source "${NS2T_PATH:?'Variable not set'}"/.env.project
set +o allexport



# ....Helper function..............................................................................
## import shell functions from norlab-shell-script-tools utilities library

TMP_CWD_BA=$(pwd)
cd "$NS2T_PATH"/src/function_library
source ./prompt_utilities.bash
source ./terminal_splash.bash
cd "$TMP_CWD_BA"

# . . Build_matrix logging functions. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# (CRITICAL) ToDo: change .log directory
BUILD_LOG_PATH=./dockerized-norlab-scripts/build_script/build_all.log
touch "$BUILD_LOG_PATH"
unset _ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS

function fetch_build_log() {
  set -o allexport; source "$BUILD_LOG_PATH"; set +o allexport
}

function agregate_build_logs() {
  _ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS=("${_ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS[@]}" "${STR_BUILD_MATRIX_SERVICES_AND_TAGS}")
}

# ====Begin========================================================================================
DOCKER_CMD=${NBS_OVERRIDE_DOCKER_CMD:-"build"}

# ....manual config................................................................................

#DOCKER_CMD="push --ignore-push-failures"
#DOCKER_CMD="up --build"
#DOCKER_CMD="build --dry-run"
#DOCKER_CMD=( build --dry-run )
#DOCKER_CMD=( build )

#export DOCKER_CONTEXT=desktop-linux
#export DOCKER_CONTEXT=jetson-nx-redleader-daemon

# ....setup........................................................................................
# Note: 'NBS_OVERRIDE_ADD_DOCKER_FLAG' is set via commandline for convenience
#DOCKER_COMMAND_W_FLAGS="build --dry-run"
DOCKER_COMMAND_W_FLAGS="$DOCKER_CMD ${NBS_OVERRIDE_ADD_DOCKER_FLAG:-""}"
SUB_SCRIPT_FLAGS=$@

# ....execute all build matrix.....................................................................
_CRAWL_BUILD_MATRIX=( "${NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY[*]:-${NBS_DOTENV_BUILD_MATRIX_ARRAY[@]}}" )

for EACH_BUILD_MATRIX in "${_CRAWL_BUILD_MATRIX[@]}" ; do

  # (CRITICAL) ToDo: refactor path to 'dn_execute_compose_over_build_matrix.bash' >> make it portable
  bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                        "${NBS_BUILD_MATRIX_CONFIG:?'Variable not set'}/$EACH_BUILD_MATRIX" \
                        ${SUB_SCRIPT_FLAGS} -- "${DOCKER_COMMAND_W_FLAGS}"

    fetch_build_log
    agregate_build_logs
done

# ....show build matrix feedback...................................................................
set -o allexport
# (CRITICAL) ToDo: refactor '.env.dockerized-norlab' >> make it portable (eg .env or set the file name via env var)
source .env.dockerized-norlab
set +o allexport
norlab_splash "${NBS_SPLASH_NAME}" "${PROJECT_GIT_REMOTE_URL}"

print_msg_done "${MSG_DIMMED_FORMAT}dn_build_all.bash${MSG_END_FORMAT} execution summary"
echo -e "${MSG_DIMMED_FORMAT}"
draw_horizontal_line_across_the_terminal_window '.'
echo -e "${MSG_END_FORMAT}"
for each_services_and_tags in "${_ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS[@]}"; do
  echo -e "${each_services_and_tags}\n"
  echo -e "${MSG_DIMMED_FORMAT}"
  draw_horizontal_line_across_the_terminal_window '.'
  echo -e "${MSG_END_FORMAT}"
done
print_formated_script_footer 'dn_build_all.bash' "${MSG_LINE_CHAR_BUILDER_LVL1}"
