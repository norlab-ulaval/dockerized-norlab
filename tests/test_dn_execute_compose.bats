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
  #load "${BATS_HELPER_PATH}/bats-detik/load" # << Kubernetes support
else
  echo -e "\n[\033[1;31mERROR\033[0m] $0 path to bats-core helper library unreachable at \"${BATS_HELPER_PATH}\"!"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Setup========================================================================================
TESTED_FILE="dn_execute_compose.bash"
TESTED_FILE_PATH="dockerized-norlab-scripts/build_script"

TEST_DOCKER_COMPOSE_FILE_PATH=dockerized-norlab-images/core-images/dependencies
TEST_DOCKER_COMPOSE_FILE="${TEST_DOCKER_COMPOSE_FILE_PATH}/docker-compose.dn-dependencies.build.yaml"

setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
#  pwd >&3 && tree -L 2 -a -hug utilities/ >&3
#  printenv >&3
}

#setup() {
#  cd "$TESTED_FILE_PATH" || exit
#}

# ====Teardown=====================================================================================

#teardown() {
#  bats_print_run_env_variable_on_error
#}

#teardown_file() {
#    echo "executed once after finishing the last test"
#}

# ====Test casses==================================================================================


@test "sourcing $TESTED_FILE from bad cwd › expect fail" {
  cd "${BATS_DOCKER_WORKDIR}/dockerized-norlab-scripts/"
  # Note:
  #  - "echo 'Y'" is for sending an keyboard input to the 'read' command which expect a single character
  #    run bash -c "echo 'Y' | bash ./function_library/$TESTED_FILE"
  #  - Alt: Use the 'yes [n]' command which optionaly send n time
  run bash -c "yes 1 | bash ./build_script/$TESTED_FILE ${TEST_DOCKER_COMPOSE_FILE}"
#  bats_print_run_env_variable
  assert_failure 1
  assert_output --partial "'$TESTED_FILE' script must be sourced from"
}


@test "sourcing $TESTED_FILE from ok cwd › expect pass" {
  cd "${BATS_DOCKER_WORKDIR}"
  run bash -c "bash ./${TESTED_FILE_PATH}/$TESTED_FILE ${TEST_DOCKER_COMPOSE_FILE}"
  assert_success
  refute_output  --partial "No such file or directory"
#  bats_print_run_env_variable
}


@test "missing docker-compose.yaml file mandatory argument › expect pass" {
  cd "${BATS_DOCKER_WORKDIR}"
  run bash -c "yes 1 | bash ./${TESTED_FILE_PATH}/$TESTED_FILE"
  assert_failure
  assert_output --partial "Missing the docker-compose.yaml file mandatory argument"

  run bash -c "yes 1 | bash ./${TESTED_FILE_PATH}/$TESTED_FILE 'docker-compose.unreachable.yaml'"
  assert_failure
  assert_output --partial "'$TESTED_FILE' can't find the docker-compose.yaml file 'docker-compose.unreachable.yaml'"
#  bats_print_run_env_variable
}

@test "docker command are passed to show_and_execute_docker" {
  local DOCKER_CMD="build --no-cache --push"
  run bash -c "bash ./${TESTED_FILE_PATH}/$TESTED_FILE ${TEST_DOCKER_COMPOSE_FILE} --fail-fast -- ${DOCKER_CMD}"
  assert_success
  assert_output --regexp .*"docker compose -f ".*"${DOCKER_CMD}"
#  bats_print_run_env_variable
}

@test "docker exit code propagation on pass › expect pass" {
  local DOCKER_CMD="version"
  mock_docker_command_exit_ok
  set +e
  source ./${TESTED_FILE_PATH}/$TESTED_FILE "${TEST_DOCKER_COMPOSE_FILE}" \
                                            --ci-test-force-runing-docker-cmd -- "$DOCKER_CMD"
  set -e
  assert_equal "$DOCKER_EXIT_CODE" 0
}

@test "docker exit code propagation on faillure › expect pass" {
  local DOCKER_CMD="version"
  mock_docker_command_exit_error
  set +e
  source ./${TESTED_FILE_PATH}/$TESTED_FILE "${TEST_DOCKER_COMPOSE_FILE}" \
                                            --ci-test-force-runing-docker-cmd -- "$DOCKER_CMD"
  set -e
  assert_equal "$DOCKER_EXIT_CODE" 1
}

@test "flags that set env variable" {
  source ./${TESTED_FILE_PATH}/$TESTED_FILE "${TEST_DOCKER_COMPOSE_FILE}" \
                                            --dockerized-norlab-version v1 \
                                            --base-image dustynv/pytorch \
                                            --os-name arbitratyName \
                                            --tag-package 2.1 \
                                            --tag-version r35.0.0 \
                                            --docker-debug-logs \
                                            --fail-fast

  assert_not_empty "$IS_TEAMCITY_RUN"
  assert_equal "${REPOSITORY_VERSION}" 'v1'
  assert_equal "${BASE_IMAGE}" 'dustynv/pytorch'
  assert_equal "${TAG_PACKAGE}" '2.1'
  assert_equal "${TAG_VERSION}" 'r35.0.0'
  assert_equal "${PROJECT_TAG}" 'arbitratyName-r35.0.0'
  assert_equal "${BUILDKIT_PROGRESS}" plain
  assert_equal "${DEPENDENCIES_BASE_IMAGE_TAG}" '2.1-r35.0.0'
  assert_equal "${DN_IMAGE_TAG}" 'DN-v1-2.1-r35.0.0'
}

@test "flag --help" {
  run bash ./${TESTED_FILE_PATH}/$TESTED_FILE "${TEST_DOCKER_COMPOSE_FILE}" \
                                                                --fail-fast \
                                                                --help
  assert_success
  assert_output --regexp .*"${TESTED_FILE} <docker-compose.yaml> \[<optional flag>\] \[-- <any docker cmd\+arg>\]".*
}

@test "flag --buildx-bake" {
  local DOCKER_CMD="--load --push --builder jetson-nx-redleader-daemon"
  run bash -c "bash ./${TESTED_FILE_PATH}/$TESTED_FILE ${TEST_DOCKER_COMPOSE_FILE} --buildx-bake --fail-fast -- ${DOCKER_CMD}"

  assert_success
  assert_output --regexp .*"docker buildx bake -f ".*"${DOCKER_CMD}"
#  bats_print_run_env_variable
}

