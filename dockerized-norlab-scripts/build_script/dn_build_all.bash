#!/bin/bash
#
# Convenient script for building all images specified in 'docker-compose.dn-dependencies.build.yaml'
#
# Usage:
#   $ bash dn_build_over_single_build_matrix.bash [<optional flag>]
#
# Arguments:
#   - [<optional flag>]   Any optional flag from 'dn_execute_compose_over_build_matrix.bash'
#
# Global
#   - Read ADD_DOCKER_FLAG    Use to quickly add docker flag at runtime
#                               e.g.: $ ADD_DOCKER_FLAG=--push --dry-run && bash dn_build_all.bash
#

clear

if [[ $( basename $(pwd) ) = build_script ]]; then
    cd ../..
elif [[ $( basename $(pwd) ) = dockerized-norlab-scripts ]]; then
    cd ..
fi

# ....Pre-condition.................................................................................
if [[ ! -f  ".env.dockerized-norlab" ]]; then
  echo -e "\n[\033[1;31mERROR\033[0m] 'dn_build_all.bash' script must be sourced from the project root!\n Curent working directory is '$(pwd)'"
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
source ./terminal_splash.bash
cd "$TMP_CWD"

# . . Build_matrix logging functions. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
BUILD_LOG_PATH=./dockerized-norlab-scripts/build_script/build_all.log
touch "$BUILD_LOG_PATH"
unset ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS

function fetch_build_log() {
  set -o allexport; source "$BUILD_LOG_PATH"; set +o allexport
}

function agregate_build_logs() {
  ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS=("${ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS[@]}" "${STR_BUILD_MATRIX_SERVICES_AND_TAGS}")
}

# ====Begin=========================================================================================
DOCKER_CMD=build
# ....manual config.................................................................................

#DOCKER_CMD="push --ignore-push-failures"
#DOCKER_CMD="up --build"
#DOCKER_CMD="build --dry-run"

#export DOCKER_CONTEXT=desktop-linux
#export DOCKER_CONTEXT=jetson-nx-redleader-daemon

# ....setup........................................................................................
# Note: 'ADD_DOCKER_FLAG' is set via commandline for convenience
DOCKER_COMMAND_W_FLAGS="$DOCKER_CMD ${ADD_DOCKER_FLAG:-""}"
SUB_SCRIPT_FLAGS=$@

# ....execute.......................................................................................
bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                      .env.build_matrix.dn-dependencies \
                      ${SUB_SCRIPT_FLAGS} -- "${DOCKER_COMMAND_W_FLAGS}"
#                      .env.build_matrix.dev \

fetch_build_log
ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS=( "${STR_BUILD_MATRIX_SERVICES_AND_TAGS}" )

bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                      .env.build_matrix.dn-control \
                      ${SUB_SCRIPT_FLAGS} -- "${DOCKER_COMMAND_W_FLAGS}"

fetch_build_log
agregate_build_logs

bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                      .env.build_matrix.dn-control-deep \
                      ${SUB_SCRIPT_FLAGS} -- "${DOCKER_COMMAND_W_FLAGS}"

fetch_build_log
agregate_build_logs

bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                      .env.build_matrix.dn-perception \
                      ${SUB_SCRIPT_FLAGS} -- "${DOCKER_COMMAND_W_FLAGS}"

fetch_build_log
agregate_build_logs

norlab_splash "${DN_SPLASH_NAME}" "${PROJECT_GIT_REMOTE_URL}"

print_msg_done "${MSG_DIMMED_FORMAT}dn_build_all.bash${MSG_END_FORMAT} execution summary"
echo -e "${MSG_DIMMED_FORMAT}"
draw_horizontal_line_across_the_terminal_window '.'
echo -e "${MSG_END_FORMAT}"
for each_services_and_tags in "${ALL_STR_BUILD_MATRIX_SERVICES_AND_TAGS[@]}"; do
  echo -e "${each_services_and_tags}\n"
  echo -e "${MSG_DIMMED_FORMAT}"
  draw_horizontal_line_across_the_terminal_window '.'
  echo -e "${MSG_END_FORMAT}"
done
print_formated_script_footer 'dn_build_all.bash' "${MSG_LINE_CHAR_BUILDER_LVL1}"
