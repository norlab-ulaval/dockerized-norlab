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

TESTED_FILE_1=".env.build_matrix.base-images-cuda-squash"
TESTED_FILE_2=".env.build_matrix.base-images-ros2"
TESTED_FILE_5=".env.build_matrix.dn-dependencies"
TESTED_FILE_3=".env.build_matrix.dn-control"
TESTED_FILE_4=".env.build_matrix.dn-control-deep"
TESTED_FILE_6=".env.build_matrix.dn-perception"
TESTED_FILE_7=".env.build_matrix.dn-project"
TESTED_FILE_PATH="build_matrix_config/prod/"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  ## Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
#  pwd >&3 && tree -L 1 -a -hug >&3
#  printenv >&3
}

# executed before each test
setup() {
  cd "$TESTED_FILE_PATH" || exit 1
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

@test "${TESTED_FILE_1} › validate env variable values › expect pass" {
  # ....Pre-condition..............................................................................
  assert_empty ${NBS_COMPOSE_DIR}
  assert_empty ${NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE}
  assert_empty ${NBS_MATRIX_ROS_DISTRO}
  assert_empty ${NBS_MATRIX_ROS_PKG}

  # ....Import N2ST library........................................................................
  source "$TESTED_FILE_1"

  # ....Tests......................................................................................
  assert_equal "${NBS_COMPOSE_DIR}" "dockerized-norlab-images/core-images/base-images"
  assert_equal "${NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE}" "${NBS_COMPOSE_DIR}/docker-compose.cuda-squash.build.yaml"
  assert_equal "(${NBS_MATRIX_ROS_DISTRO[*]})" "(none)"
  assert_equal "(${NBS_MATRIX_ROS_PKG[*]})" "(none)"
}

@test "${TESTED_FILE_2} › validate env variable values › expect pass" {
  # ....Pre-condition..............................................................................
  assert_empty ${NBS_COMPOSE_DIR}
  assert_empty ${NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE}
  assert_empty ${NBS_MATRIX_ROS_DISTRO}
  assert_empty ${NBS_MATRIX_ROS_PKG}

  # ....Import N2ST library........................................................................
  source "$TESTED_FILE_2"

  # ....Tests......................................................................................
  assert_equal "${NBS_COMPOSE_DIR}" "dockerized-norlab-images/core-images/base-images"
  assert_equal "${NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE}" "${NBS_COMPOSE_DIR}/docker-compose.ros2.build.yaml"
  assert_empty ${NBS_MATRIX_ROS_DISTRO}
  assert_empty ${NBS_MATRIX_ROS_PKG}

}
