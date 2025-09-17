#!/bin/bash
# ==================================================================================================
# dn::show_debug_build_information
#
# This shell function outputs project, environment and system details intended for use in debugging
# the Dockerized Norlab build process. It prints a plethora of values for variables associated with
# different components, paths, and settings of the build.
#
# Usage:
#   Add or modify arguments to the function call if desired,
#     - Set SHOW_TREE to true to print tree to stdin
#     - Set BREAKPOINT_BEAVIOUR to true to let the fct behave as a breakpoint
#   then
#   $ dn::show_debug_build_information
#
#
# Globals:
#   Read:
#     DN_*, TEAMCITY_*, NBS_*, N2ST_*, _REPO_ROOT, _PATH_TO_SCRIPT, PATH, PWD, OLDPWD, HOME
#
# Arguments: None.
#
# Outputs:
#   This function prints the value of various environment variables and project details to stdout.
#
# Returns:
#   The function returns an exit status of 0, or exits with an error message if
#   the breakpoint behaviour is set to true.
# ===================================================================================================
function dn::show_debug_build_information() {


  # ....Manual setting.............................................................................
  # Set to true to print tree to stdin
  local SHOW_TREE=false
  # Set to true to let the fct behave as a breakpoint
  local BREAKPOINT_BEAVIOUR=false

  # ....Begin......................................................................................
  n2st::teamcity_service_msg_blockOpened "DN show_debug_build_information"
  echo -e "\n==============================================================================================="
  echo -e "===Debug breakpoint============================================================================\n"
  if [[ ${SHOW_TREE} == true ]]; then
    tree -a -L 1 "${DN_PATH}"
    echo -e "\n==============================================================================================="
    echo -e "===============================================================================================\n"
    tree -a "${DN_PATH}/dockerized-norlab-images"
    tree -a "${DN_PATH}/dockerized-norlab-scripts"
    echo -e "\n==============================================================================================="
    echo -e "===============================================================================================\n"
  fi
  echo -e "Explicit cmd"
  local DEBUG_PROJECT_GIT_REMOTE_URL_HARDCODED
  local DEBUG_PROJECT_GIT_REMOTE_URL
  local DEBUG_PROJECT_GIT_NAME_HARDCODED
  local DEBUG_PROJECT_GIT_NAME
  local DEBUG_PROJECT_PATH
  local DEBUG_PROJECT_SRC_NAME
  DEBUG_PROJECT_GIT_REMOTE_URL_HARDCODED="https://github.com/norlab-ulaval/dockerized-norlab"
  DEBUG_PROJECT_GIT_REMOTE_URL=$(git remote get-url origin)
  DEBUG_PROJECT_GIT_NAME_HARDCODED=$(basename "${DEBUG_PROJECT_GIT_REMOTE_URL_HARDCODED}" .git)
  DEBUG_PROJECT_GIT_NAME=$(basename "${DEBUG_PROJECT_GIT_REMOTE_URL}" .git)
  DEBUG_PROJECT_PATH=$(git rev-parse --show-toplevel)
  DEBUG_PROJECT_SRC_NAME="$(basename "${DEBUG_PROJECT_PATH}")"
  echo -e "DEBUG_PROJECT_GIT_REMOTE_URL_HARDCODED=${DEBUG_PROJECT_GIT_REMOTE_URL_HARDCODED}"
  echo -e "DEBUG_PROJECT_GIT_REMOTE_URL=${DEBUG_PROJECT_GIT_REMOTE_URL}"
  echo -e "DEBUG_PROJECT_GIT_NAME_HARDCODED=${DEBUG_PROJECT_GIT_NAME_HARDCODED}"
  echo -e "DEBUG_PROJECT_GIT_NAME=${DEBUG_PROJECT_GIT_NAME}"
  echo -e "DEBUG_PROJECT_PATH=${DEBUG_PROJECT_PATH}"
  echo -e "DEBUG_PROJECT_SRC_NAME=${DEBUG_PROJECT_SRC_NAME}"
  echo -e "\n==============================================================================================="
  echo -e "===============================================================================================\n"
  echo "DN_IMAGE_TAG=$(printenv DN_IMAGE_TAG)"
  echo "DN_PROJECT_GID=$(printenv DN_PROJECT_GID)"
  echo "DN_PROJECT_COMPOSE_NAME=$(printenv DN_PROJECT_COMPOSE_NAME)"
  echo "DN_PROJECT_UID=$(printenv DN_PROJECT_UID)"
  echo "DN_CONTAINER_NAME=$(printenv DN_CONTAINER_NAME)"
  echo "DN_SPLASH_NAME=$(printenv DN_SPLASH_NAME)"
  echo "DN_PROJECT_BASE_IMG=$(printenv DN_PROJECT_BASE_IMG)"
  echo "DN_PROJECT_HUB=$(printenv DN_PROJECT_HUB)"
  echo "DN_PROJECT_IMAGE_NAME=$(printenv DN_PROJECT_IMAGE_NAME)"
  echo "DN_SRC_NAME=$(printenv DN_SRC_NAME)"
  echo "DN_COMPOSE_PLATFORMS=$(printenv DN_COMPOSE_PLATFORMS)"
  echo "DN_GIT_NAME=$(printenv DN_GIT_NAME)"
  echo "DN_IMPORTED=$(printenv DN_IMPORTED)"
  echo "DN_PROMPT_NAME=$(printenv DN_PROMPT_NAME)"
  echo "DN_PROJECT_USER=$(printenv DN_PROJECT_USER)"
  echo "DN_PATH=$(printenv DN_PATH)"
  echo "DN_PROJECT_GIT_NAME=$(printenv DN_PROJECT_GIT_NAME)"
  echo "DN_TARGET_DEVICE=$(printenv DN_TARGET_DEVICE)"
  echo "DN_GIT_REMOTE_URL=$(printenv DN_GIT_REMOTE_URL)"
  echo "DN_HUB=$(printenv DN_HUB)"
  echo "DN_PROJECT_GIT_DOMAIN=$(printenv DN_PROJECT_GIT_DOMAIN)"
  echo "DN_IMAGE_TAG_NO_ROS=$(printenv DN_IMAGE_TAG_NO_ROS)"
  echo "IS_TEAMCITY_RUN=$(printenv IS_TEAMCITY_RUN)"
  echo -e "\n===============================================================================================\n"
  echo "TEAMCITY_GIT_PATH=$(printenv TEAMCITY_GIT_PATH)"
  echo "TEAMCITY_BUILD_PROPERTIES_FILE=$(printenv TEAMCITY_BUILD_PROPERTIES_FILE)"
  echo "TEAMCITY_GIT_VERSION=$(printenv TEAMCITY_GIT_VERSION)"
  echo "TEAMCITY_CAPTURE_ENV=$(printenv TEAMCITY_CAPTURE_ENV)"
  echo "TEAMCITY_PROCESS_PARENT_FLOW_ID=$(printenv TEAMCITY_PROCESS_PARENT_FLOW_ID)"
  echo "TEAMCITY_PROCESS_FLOW_ID=$(printenv TEAMCITY_PROCESS_FLOW_ID)"
  echo "TEAMCITY_BUILDCONF_NAME=$(printenv TEAMCITY_BUILDCONF_NAME)"
  echo "TEAMCITY_VERSION=$(printenv TEAMCITY_VERSION)"
  echo "TEAMCITY_PROJECT_NAME=$(printenv TEAMCITY_PROJECT_NAME)"
  echo -e "\n===============================================================================================\n"
  echo "NBS_TMP_TEST_LIB_SOURCING_ENV_EXPORT=$(printenv NBS_TMP_TEST_LIB_SOURCING_ENV_EXPORT)"
  echo "NBS_BUILD_MATRIX_CONFIG=$(printenv NBS_BUILD_MATRIX_CONFIG)"
  echo "NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE=$(printenv NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE)"
  echo "NBS_GIT_REMOTE_URL=$(printenv NBS_GIT_REMOTE_URL)"
  echo "NBS_GIT_NAME=$(printenv NBS_GIT_NAME)"
  echo "NBS_COMPOSE_DIR=$(printenv NBS_COMPOSE_DIR)"
  echo "NBS_SRC_NAME=$(printenv NBS_SRC_NAME)"
  echo "NBS_PATH=$(printenv NBS_PATH)"
  echo "NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG=$(printenv NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG)"
  echo "NBS_SPLASH_NAME=$(printenv NBS_SPLASH_NAME)"
  echo "NBS_PROMPT_NAME=$(printenv NBS_PROMPT_NAME)"
  echo "NBS_IMPORTED=$(printenv NBS_IMPORTED)"
  echo -e "\n===============================================================================================\n"
  echo "N2ST_GIT_NAME=$(printenv N2ST_GIT_NAME)"
  echo "N2ST_PROMPT_NAME=$(printenv N2ST_PROMPT_NAME)"
  echo "N2ST_PATH=$(printenv N2ST_PATH)"
  echo "N2ST_SRC_NAME=$(printenv N2ST_SRC_NAME)"
  echo "N2ST_GIT_REMOTE_URL=$(printenv N2ST_GIT_REMOTE_URL)"
  echo -e "\n===============================================================================================\n"
  echo "PATH=$(printenv PATH)"
  echo -e "\n===============================================================================================\n"
  echo "PWD=$(printenv PWD)"
  echo "OLDPWD=$(printenv OLDPWD)"
  echo "HOME=$(printenv HOME)"
  echo -e "\n===============================================================================================\n"
  echo "_REPO_ROOT=$(printenv _REPO_ROOT)"
  echo "_PATH_TO_SCRIPT=$(printenv _PATH_TO_SCRIPT)"
  echo -e "\n============================================================================Debug breakpoint==="
  echo -e "===============================================================================================\n"
  n2st::teamcity_service_msg_blockClosed_v2 "DN show_debug_build_information"

  if [[ ${BREAKPOINT_BEAVIOUR} == true ]]; then
    n2st::print_msg_error_and_exit "debug breakpoint"
  fi

  return 0
}
