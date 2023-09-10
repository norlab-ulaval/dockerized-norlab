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
  load "bats_testing_tools/bats_helper_functions"
  load "bats_testing_tools/bats_helper_functions_local"
  #load "${BATS_HELPER_PATH}/bats-detik/load" # << Kubernetes support
else
  echo -e "\n[\033[1;31mERROR\033[0m] $0 path to bats-core helper library unreachable at \"${BATS_HELPER_PATH}\"!"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi



# ====Setup=========================================================================================
TESTED_FILE="dn_execute_compose_over_build_matrix.bash"
TESTED_FILE_PATH="dockerized-norlab-scripts/build_script"

setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
#  pwd >&3 && tree -L 2 -a -hug utilities/ >&3
#  printenv >&3
}

#setup() {
#  cd "$TESTED_FILE_PATH" || exit
#}

# ====Teardown======================================================================================

#teardown() {
#  bats_print_run_env_variable_on_error
##  bats_print_run_env_variable
#}

#teardown_file() {
#    echo "executed once after finishing the last test"
#}

# ====Test casses===================================================================================

@test "sourcing $TESTED_FILE from bad cwd › expect fail" {
  cd "${BATS_DOCKER_WORKDIR}/dockerized-norlab-scripts/"
  # Note:
  #  - "echo 'Y'" is for sending an keyboard input to the 'read' command which expect a single character
  #    run bash -c "echo 'Y' | bash ./function_library/$TESTED_FILE"
  #  - Alt: Use the 'yes [n]' command which optionaly send n time
  run bash -c "yes 1 | bash ./build_script/$TESTED_FILE 'tests/.env.build_matrix.mock'"
#  bats_print_run_env_variable
  assert_failure 1
  assert_output --partial "'$TESTED_FILE' script must be sourced from"
}


@test "sourcing $TESTED_FILE from ok cwd › expect pass" {
  cd "${BATS_DOCKER_WORKDIR}"
  run bash -c "bash ./${TESTED_FILE_PATH}/$TESTED_FILE 'tests/.env.build_matrix.mock'"
  assert_success
  refute_output  --partial "No such file or directory"
#  bats_print_run_env_variable
}

@test "missing dotenv build matrix file mandatory argument› expect pass" {
  cd "${BATS_DOCKER_WORKDIR}"
  run bash -c "yes 1 | bash ./${TESTED_FILE_PATH}/$TESTED_FILE"
  assert_failure
  assert_output --partial "Missing the dotenv build matrix file mandatory argument"

  run bash -c "yes 1 | bash ./${TESTED_FILE_PATH}/$TESTED_FILE '.env.build_matrix.unreachable'"
  assert_failure
  assert_output --partial "'$TESTED_FILE' can't find dotenv build matrix file '.env.build_matrix.unreachable'"
#  bats_print_run_env_variable
}

@test "docker command are passed to show_and_execute_docker" {
  local DOCKER_CMD="build --no-cache --push"
  run bash -c "bash ./${TESTED_FILE_PATH}/$TESTED_FILE 'tests/.env.build_matrix.mock' -- ${DOCKER_CMD}"
  assert_success
  assert_output --regexp .*"Skipping the execution of Docker command".*
  assert_output --regexp .*"docker compose -f ".*"${DOCKER_CMD}".*
  assert_output --regexp .*"with command".*"${DOCKER_CMD}".*
}

@test "dotenv build matrix file argument › ok" {
  local DOCKER_CMD="version"
  set +e
  mock_docker_command_exit_ok
  run source ./${TESTED_FILE_PATH}/$TESTED_FILE 'tests/.env.build_matrix.mock' \
                                                --ci-test-force-runing-docker-cmd \
                                                -- "$DOCKER_CMD"
  set -e
  assert_success
  assert_output --regexp .*"DN_MATRIX_L4T_SUPPORTED_VERSIONS\=\(r11.1.1 r22.2.2\)".*
#  bats_print_run_env_variable
}

@test "docker exit code propagation on pass › expect pass" {
  local DOCKER_CMD="version"
  set +e
  mock_docker_command_exit_ok
  run source ./${TESTED_FILE_PATH}/$TESTED_FILE 'tests/.env.build_matrix.mock' \
                                                --ci-test-force-runing-docker-cmd \
                                                -- "$DOCKER_CMD"
  set -e
  assert_success
  assert_output --regexp .*"Pass".*"DNlatest-JC-humble-ros-core-l4t-r11.1.1".*
  assert_output --regexp .*"Pass".*"DNlatest-JC-humble-pytorch-l4t-r11.1.1".*
  assert_output --regexp .*"Pass".*"DNlatest-JC-humble-pytorch-l4t-r22.2.2".*
  assert_output --regexp .*"Pass".*"DNlatest-JC-humble-pytorch-l4t-r22.2.2".*
  assert_output --regexp .*"Pass".*"DNv9.9.9-JC-humble-ros-core-l4t-r11.1.1".*
  assert_output --regexp .*"Pass".*"DNv9.9.9-JC-humble-pytorch-l4t-r11.1.1".*
  assert_output --regexp .*"Pass".*"DNv9.9.9-JC-humble-pytorch-l4t-r22.2.2".*
  assert_output --regexp .*"Pass".*"DNv9.9.9-JC-humble-pytorch-l4t-r22.2.2".*
#  bats_print_run_env_variable
}

@test "docker exit code propagation on faillure › expect pass" {
  local DOCKER_CMD="version"
  mock_docker_command_exit_error
  fake_IS_TEAMCITY_RUN
  set +e
  run source ./${TESTED_FILE_PATH}/$TESTED_FILE 'tests/.env.build_matrix.mock' \
                                                --ci-test-force-runing-docker-cmd \
                                                -- "$DOCKER_CMD"
  set -e
  assert_failure
  assert_output --regexp .*"Fail".*"DNlatest-JC-humble-ros-core-l4t-r11.1.1".*
  assert_output --regexp .*"Fail".*"DNlatest-JC-humble-pytorch-l4t-r11.1.1".*
  assert_output --regexp .*"Fail".*"DNlatest-JC-humble-pytorch-l4t-r22.2.2".*
  assert_output --regexp .*"Fail".*"DNlatest-JC-humble-pytorch-l4t-r22.2.2".*
  assert_output --regexp .*"Fail".*"DNv9.9.9-JC-humble-ros-core-l4t-r11.1.1".*
  assert_output --regexp .*"Fail".*"DNv9.9.9-JC-humble-pytorch-l4t-r11.1.1".*
  assert_output --regexp .*"Fail".*"DNv9.9.9-JC-humble-pytorch-l4t-r22.2.2".*
  assert_output --regexp .*"Fail".*"DNv9.9.9-JC-humble-pytorch-l4t-r22.2.2".*
}

@test "docker exit code propagation on faillure › expect pass (TeamCity casses)" {
  local DOCKER_CMD="version"
  mock_docker_command_exit_error
  fake_IS_TEAMCITY_RUN
  set +e
  run source ./${TESTED_FILE_PATH}/$TESTED_FILE 'tests/.env.build_matrix.mock' \
                                                --ci-test-force-runing-docker-cmd \
                                                -- "$DOCKER_CMD"
  set -e
  assert_failure
  assert_output --regexp "\#\#teamcity\[buildProblem description='BUILD FAIL with docker exit code: 1'\]"
#  bats_print_run_env_variable
}

@test "flags that set env variable" {
  set +e
  mock_docker_command_exit_ok
  run source ./${TESTED_FILE_PATH}/$TESTED_FILE 'tests/.env.build_matrix.mock' \
                                                --ci-test-force-runing-docker-cmd \
                                                --dockerized-norlab-version-build-matrix-override 'v8.8.8' \
                                                --os-name-build-matrix-override 'l4t' \
                                                --l4t-version-build-matrix-override 'r33.3.3'
  set -e
  assert_output --regexp .*"Pass".*"DNv8.8.8-JC-humble-ros-core-l4t-r33.3.3".*
  assert_output --regexp .*"Pass".*"DNv8.8.8-JC-humble-pytorch-l4t-r33.3.3".*

  refute_output --regexp .*"Pass".*"DNv9.9.9-JC-humble-ros-core-l4t-r11.1.1".*
  refute_output --regexp .*"Pass".*"DNv9.9.9-JC-humble-pytorch-l4t-r11.1.1".*
}

@test "flag --help" {
  run bash ./${TESTED_FILE_PATH}/$TESTED_FILE 'tests/.env.build_matrix.mock' --help
  assert_success
  assert_output --regexp .*"${TESTED_FILE} '<.env.build_matrix.*>' \[<optional flag>\] \[-- <any docker cmd\+arg>\]".*
}
