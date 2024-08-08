#!/bin/bash
# =================================================================================================
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
#   - Read NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY  Use to quickly override the build matrix list
#             e.g.: $ NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY=( '.env.build_matrix.dev' ) && source dn_build_all.bash
#   - Read STR_BUILD_MATRIX_SERVICES_AND_TAGS from build_all.log
#
# =================================================================================================

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
source ".env.build_matrix.main"
if [[ -n ${NBS_OVERRIDE_BUILD_MATRIX_MAIN} ]]; then
  # Note: Override values from .env.build_matrix.main
  source "${NBS_OVERRIDE_BUILD_MATRIX_MAIN}"
fi
set +o allexport


# . . Build_matrix logging functions. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# (☕MINOR) ToDo: change .log directory
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
if [[ -n $(set | grep -e NBS_OVERRIDE | sed 's/ZSH_EXECUTION_STRING.*//') ]]; then
  n2st::print_msg "Using the folowing environment variable override\n"
  echo -e "${MSG_DIMMED_FORMAT}$(set | grep -e NBS_OVERRIDE | sed 's/ZSH_EXECUTION_STRING.*//' | sed 's/_p9k__.*//' | sed 's/\[.*\]=//' | sed 's/^NBS_OVERRIDE/    NBS_OVERRIDE/')${MSG_END_FORMAT}"
  echo
fi

# ....setup........................................................................................
# Note: 'NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG' is set via commandline for convenience
DOCKER_COMMAND_W_FLAGS="${NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG:-"build"}"

# ....execute all build matrix.....................................................................
_CRAWL_BUILD_MATRIX=( "${NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY[@]:-${NBS_DOTENV_BUILD_MATRIX_ARRAY[@]:?err}}" )

for EACH_BUILD_MATRIX in "${_CRAWL_BUILD_MATRIX[@]}" ; do

  n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}

    bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash
                 ${NBS_BUILD_MATRIX_CONFIG:?'Variable not set'}/$EACH_BUILD_MATRIX
                 $* -- ${DOCKER_COMMAND_W_FLAGS}
${MSG_END_FORMAT}"


  cd "${DN_PATH:?'Variable not set'}" || exit 1
  bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                        "${NBS_BUILD_MATRIX_CONFIG:?'Variable not set'}/$EACH_BUILD_MATRIX" \
                        "$@" -- "${DOCKER_COMMAND_W_FLAGS}"

    dn::fetch_build_log
    dn::agregate_build_logs
done

# ....show build matrix feedback...................................................................
set -o allexport
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

n2st::print_formated_script_footer 'dn_build_all.bash' "${MSG_LINE_CHAR_BUILDER_LVL1}"

# ====Teardown=====================================================================================
cd "${DN_PATH:?'Variable not set'}" || exit 1

if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  if [[ $RAISE_ERROR == true ]]; then
    exit 1
  else
    exit 0
  fi
else
  # This script is being sourced, ie: __name__="__source__"
  if [[ $RAISE_ERROR == true ]]; then
    return 1
  else
    return 0
  fi
fi



