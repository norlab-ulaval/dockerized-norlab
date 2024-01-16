#!/usr/bin/env bats
#
# Usage in docker container
#   $ REPO_ROOT=$(pwd) && RUN_TESTS_IN_DIR='tests'
#   $ docker run -it --rm -v "$REPO_ROOT:/code" bats/bats:latest "$RUN_TESTS_IN_DIR"
#
#   Note: "/code" is the working directory in the bats official image
#
# bats-core ref:
#   - https://bats-core.readthedocs.io/en/stable/tutorial.html
#   - https://bats-core.readthedocs.io/en/stable/writing-tests.html
#   - https://opensource.com/article/19/2/testing-bash-bats
#       ↳ https://github.com/dmlond/how_to_bats/blob/master/test/build.bats
#
# Helper library: 
#   - https://github.com/bats-core/bats-assert
#   - https://github.com/bats-core/bats-support
#   - https://github.com/bats-core/bats-file
#

BATS_HELPER_PATH=/usr/lib/bats
if [[ -d ${BATS_HELPER_PATH} ]]; then
  load "${BATS_HELPER_PATH}/bats-support/load"
  load "${BATS_HELPER_PATH}/bats-assert/load"
  load "${BATS_HELPER_PATH}/bats-file/load"
  load "${SRC_CODE_PATH}/${N2ST_BATS_TESTING_TOOLS_RELATIVE_PATH}/bats_helper_functions"
  #load "${BATS_HELPER_PATH}/bats-detik/load" # << Kubernetes support
else
  echo -e "\n[\033[1;31mERROR\033[0m] $0 path to bats-core helper library unreachable at \"${BATS_HELPER_PATH}\"!" 1>&2
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Setup========================================================================================

TESTED_FILE="import_dockerized_norlab_tools.bash"
TESTED_FILE_PATH="./"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  ## Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
#  pwd >&3 && tree -L 1 -a -hug >&3
#  printenv >&3
}

# executed before each test
setup() {
  cd "${BATS_DOCKER_WORKDIR}/$TESTED_FILE_PATH" || exit 1
#  cd "$TESTED_FILE_PATH" || exit 1
}

# ====Teardown=====================================================================================

# executed after each test
teardown() {
  bats_print_run_env_variable_on_error
}

## executed once after finishing the last test (valide for all test in that file)
#teardown_file() {
#}

# ====Test casses==================================================================================

@test "assess execute with \"source $TESTED_FILE\" › expect pass" {

  assert_dir_exist "/code/dockerized-norlab"
  assert_file_exist "/code/dockerized-norlab/.env.dockerized-norlab-build-system"
  assert_file_exist "/code/dockerized-norlab/.env.dockerized-norlab-project"

  assert_dir_exist "/code/dockerized-norlab/utilities/norlab-shell-script-tools"
  assert_file_exist "/code/dockerized-norlab/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"

  run source "$TESTED_FILE"
  assert_success
}

@test "Test utilities › validate env var are not set between test run" {
  assert_empty "${DN_PATH}"
  assert_empty "${N2ST_PATH}"
  assert_empty "${NBS_PATH}"
  assert_empty "${DN_IMPORTED}"
}

@test "${TESTED_FILE} › check if .env.dockerized-norlab-build-system was properly sourced › expect pass" {
  # ....Pre-condition..............................................................................
  assert_empty ${PROJECT_PROMPT_NAME}
  assert_empty ${PROJECT_GIT_REMOTE_URL}
  assert_empty ${PROJECT_GIT_NAME}
  assert_empty ${PROJECT_SRC_NAME}
  assert_empty ${PROJECT_PATH}

  assert_empty ${DN_PROMPT_NAME}
  assert_empty ${DN_GIT_REMOTE_URL}
  assert_empty ${DN_GIT_NAME}
  assert_empty ${DN_SRC_NAME}
  assert_empty ${DN_PATH}

  # ....Import N2ST library........................................................................
  source "$TESTED_FILE"

  # ....Tests......................................................................................
  assert_equal "${DN_PROMPT_NAME}" "DN-build-system"
  assert_regex "${DN_GIT_REMOTE_URL}" "https://github.com/norlab-ulaval/dockerized-norlab"'(".git")?'
  assert_equal "${DN_GIT_NAME}" "dockerized-norlab"
  assert_equal "${DN_SRC_NAME}" "dockerized-norlab"
  assert_equal "${DN_PATH}" "/code/dockerized-norlab"

  assert_equal "${NBS_BUILD_MATRIX_CONFIG}" "/code/dockerized-norlab/build_matrix_config"
  assert_equal "${NBS_PATH}" "/code/dockerized-norlab/utilities/norlab-build-system"
  assert_equal "${N2ST_PATH}" "/code/dockerized-norlab/utilities/norlab-shell-script-tools"
}

@test "${TESTED_FILE} › check if .env.dockerized-norlab-project was properly sourced › expect pass" {
  # ....Pre-condition..............................................................................
  assert_empty ${DN_HUB}
  assert_empty ${DN_COMPOSE_NAME}
  assert_empty ${DN_SPLASH_NAME}
  assert_empty ${NBS_SPLASH_NAME}
  assert_empty ${DN_PROJECT_HUB}
  assert_empty ${DN_PROJECT_GIT_DOMAIN}
  assert_empty ${DN_PROJECT_IMAGE_NAME}
  assert_empty ${DN_PROJECT_BASE_IMG}
  assert_empty ${DN_PROJECT_GIT_NAME}
  assert_empty ${DN_PROJECT_COMPOSE_NAME}


  # ....Import N2ST library........................................................................
  source "$TESTED_FILE"

  # ....Tests......................................................................................
  assert_equal "${DN_HUB}" "norlabulaval"
  assert_equal "${DN_COMPOSE_NAME}" "dockerized-norlab"
  assert_equal "${DN_SPLASH_NAME}" "Dockerized-NorLab"
  assert_equal "${NBS_SPLASH_NAME}" "Dockerized-NorLab"
  assert_equal "${DN_PROJECT_HUB}" "norlabulaval"
  assert_equal "${DN_PROJECT_GIT_DOMAIN}" "norlab-ulaval"
  assert_equal "${DN_PROJECT_IMAGE_NAME}" "dockerized-norlab-project-template"
  assert_equal "${DN_PROJECT_BASE_IMG}" "dn-control-deep-rl-f1tenth"
  assert_equal "${DN_PROJECT_GIT_NAME}" "dockerized-norlab-project-mock"
  assert_equal "${DN_PROJECT_COMPOSE_NAME}" "dockerized-norlab-project-mock"
}

@test "${TESTED_FILE} › check N2ST lib import › expect pass" {
  source "$TESTED_FILE"
  run n2st::print_msg_done "Test N2ST import"

  assert_success
  assert_output --regexp .*"\[DN-build-system done\]".*"Test N2ST import"

  assert_equal "${N2ST_PROMPT_NAME}" "N2ST"
  assert_regex "${N2ST_GIT_REMOTE_URL}" "https://github.com/norlab-ulaval/norlab-shell-script-tools"'(".git")?'
  assert_equal "${N2ST_GIT_NAME}" "norlab-shell-script-tools"
  assert_equal "${N2ST_SRC_NAME}" "norlab-shell-script-tools"
  assert_equal "${N2ST_PATH}" "/code/dockerized-norlab/utilities/norlab-shell-script-tools"
}

@test "${TESTED_FILE} › check NBS lib import › expect pass" {
  source "$TESTED_FILE"

  run nbs::test_export_fct
  assert_success

  assert_equal "${NBS_PROMPT_NAME}" "NBS"
  assert_regex "${NBS_GIT_REMOTE_URL}" "https://github.com/norlab-ulaval/norlab-build-system"'(".git")?'
  assert_equal "${NBS_GIT_NAME}" "norlab-build-system"
  assert_equal "${NBS_SRC_NAME}" "norlab-build-system"
  assert_equal "${NBS_PATH}" "/code/dockerized-norlab/utilities/norlab-build-system"
}

@test "${TESTED_FILE} › validate teardown › expect pass" {

  assert_equal "$(pwd)" "$( realpath "/code/dockerized-norlab/${TESTED_FILE_PATH}" )"

  run source $TESTED_FILE
  assert_success

  assert_equal "$(pwd)" "$( realpath "/code/dockerized-norlab/${TESTED_FILE_PATH}" )"

}

@test "${TESTED_FILE} › validate the import function mechanism › expect pass" {

  cd "${SRC_CODE_PATH}"
  source "$TESTED_FILE"

  run nbs::execute_compose --help

  assert_empty "${NBS_TMP_TEST_LIB_SOURCING_FUNC}"
  nbs::test_export_fct
  assert_not_empty "${NBS_TMP_TEST_LIB_SOURCING_FUNC}"

#  run printenv >&3
  run printenv
  assert_success
  assert_output --partial "NBS_TMP_TEST_LIB_SOURCING_FUNC=Let it SNOW"
}

@test "assess execute with \"bash $TESTED_FILE\" › expect fail" {
  run bash "$TESTED_FILE"
  assert_failure
  assert_output --regexp "\[ERROR\]".*"This script must be sourced i.e.:".*"source".*"$TESTED_FILE"
}
