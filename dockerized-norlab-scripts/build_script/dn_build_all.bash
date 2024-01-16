#!/bin/bash
# =================================================================================================
#
# Convenient script for building all images by crawling over all ".env.build_matrix.dn-*" file
#
# Usage:

#   $ [bash|source] dn_build_all.bash [<optional flag>]
#
# Arguments:
#   - [<optional flag>]   Any optional flag from 'dn_execute_compose_over_build_matrix.bash'
#
# Global
#   - Read NBS_OVERRIDE_BUILD_MATRIX_MAIN          Use to quickly change the .env.build_matrix.main file
#   - Read NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG            Use to quickly add docker flag at runtime
#             e.g.: $ NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG="build --push --dry-run" && source dn_build_all.bash
#   - Read NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY  Use to quickly overide the build matrix list
#             e.g.: $ NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY=( '.env.build_matrix.dev' ) && source dn_build_all.bash
#   - Read STR_BUILD_MATRIX_SERVICES_AND_TAGS from build_all.log
#
# =================================================================================================
#clear
#set -x # (CRITICAL) ToDo: on task end >> mute this line ←

#SUB_SCRIPT_FLAGS=$@

# ....Pre-condition................................................................................

if [[ ! -f  ".env.dockerized-norlab-build-system" ]]; then
  echo -e "\n[\033[1;31mERROR\033[0m] 'dn_build_all.bash' script must be executed from the project root!\n Curent working directory is '$(pwd)'"  1>&2
  exit 1
fi

# ....Load environment variables from file.........................................................

#
# The main .env.build_matrix to load
#
NBS_BUILD_MATRIX_MAIN=${NBS_OVERRIDE_BUILD_MATRIX_MAIN:-".env.build_matrix.main"}

set -o allexport
source .env.dockerized-norlab-build-system || exit 1
source "$NBS_BUILD_MATRIX_MAIN"

# Set PROJECT_GIT_REMOTE_URL
source "${N2ST_PATH:?'Variable not set'}"/.env.project || exit 1

cd "${NBS_PATH:?'Variable not set'}"  || exit 1
source import_norlab_build_system_lib.bash || exit 1

cd "${DN_PATH:?'Variable not set'}" || exit 1
set +o allexport


# . . Build_matrix logging functions. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# (CRITICAL) ToDo: change .log directory
BUILD_LOG_PATH=./dockerized-norlab-scripts/build_script/build_all.log
touch "$BUILD_LOG_PATH"
unset _ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS

function dn::fetch_build_log() {
  set -o allexport; source "$BUILD_LOG_PATH"; set +o allexport
}

function dn::agregate_build_logs() {
  _ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS=("${_ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS[@]}" "${STR_BUILD_MATRIX_SERVICES_AND_TAGS}")
}

# ====Begin========================================================================================

# ....manual config................................................................................

## ToDo: on task end >> delete next bloc ↓↓
#NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG="push --ignore-push-failures"
#NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG="up --build"
#NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG="build --dry-run"
#NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG=( build --dry-run )
#NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG=( build )
#
#export DOCKER_CONTEXT=desktop-linux
#export DOCKER_CONTEXT=jetson-nx-redleader-daemon

# ....setup........................................................................................
# Note: 'NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG' is set via commandline for convenience
DOCKER_COMMAND_W_FLAGS="${NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG:-"build"}"

# ....execute all build matrix.....................................................................
_CRAWL_BUILD_MATRIX=( "${NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY[*]:-${NBS_DOTENV_BUILD_MATRIX_ARRAY[@]}}" )

for EACH_BUILD_MATRIX in "${_CRAWL_BUILD_MATRIX[@]}" ; do

  echo "bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash
                               ${NBS_BUILD_MATRIX_CONFIG:?'Variable not set'}/$EACH_BUILD_MATRIX
                               $* -- ${DOCKER_COMMAND_W_FLAGS}"


  cd "${DN_PATH:?'Variable not set'}" || exit 1
  bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                        "${NBS_BUILD_MATRIX_CONFIG:?'Variable not set'}/$EACH_BUILD_MATRIX" \
                        $@ -- "${DOCKER_COMMAND_W_FLAGS}"

    dn::fetch_build_log
    dn::agregate_build_logs
done

# ....show build matrix feedback...................................................................
set -o allexport
# (CRITICAL) ToDo: refactor '.env.dockerized-norlab-project' >> make it portable (eg .env or set the file name via env var)
source .env.dockerized-norlab-project
set +o allexport
n2st::norlab_splash "${NBS_SPLASH_NAME}" "${PROJECT_GIT_REMOTE_URL}"


n2st::print_msg_done "${MSG_DIMMED_FORMAT}dn_build_all.bash${MSG_END_FORMAT} execution summary"
echo -e "${MSG_DIMMED_FORMAT}"
n2st::draw_horizontal_line_across_the_terminal_window '.'
echo -e "${MSG_END_FORMAT}"

RAISE_ERROR=false
for each_services_and_tags in "${_ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS[@]}"; do
  echo -e "${each_services_and_tags}\n"
  echo -e "${MSG_DIMMED_FORMAT}"
  n2st::draw_horizontal_line_across_the_terminal_window '.'
  echo -e "${MSG_END_FORMAT}"
  if [[ "${each_services_and_tags}" == *'Fail'*  ]]; then
    RAISE_ERROR=true
  fi
done

# ====Teardown=====================================================================================
n2st::print_formated_script_footer 'dn_build_all.bash' "${MSG_LINE_CHAR_BUILDER_LVL1}"

cd "${DN_PATH:?'Variable not set'}" || exit 1

if [[ $RAISE_ERROR == true ]]; then
  exit 1
else
  exit 0
fi


