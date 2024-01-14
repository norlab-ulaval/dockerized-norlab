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

setup() {
#  cd "$TESTED_FILE_PATH" || exit 1

  NBS_PATH=${BATS_DOCKER_WORKDIR}/utilities/norlab-build-system
  cd "${NBS_PATH}"
  source "import_norlab_build_system_lib.bash" || exit 1

  cd "${BATS_DOCKER_WORKDIR}" || exit 1
}

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

  source ./build_script/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE}

  assert_failure 1
  assert_output --regexp .*"\[".*"DN ERROR".*"\]".*"'dn::execute_compose' function must be executed from the project root!"
}

@test "sourcing $TESTED_FILE from ok cwd › expect pass" {

  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE}

  assert_success
  refute_output --regexp .*"\[".*"DN ERROR".*"\]".*"'dn::execute_compose' function must be executed from the project root!"
#  bats_print_run_env_variable
}

@test "missing docker-compose.yaml file mandatory argument › expect fail" {

  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose

  assert_output --partial 'Missing the docker-compose.yaml file mandatory argument'

  run dn::execute_compose "docker-compose.unreachable.yaml"
  assert_failure
  assert_output --regexp .*"\[".*"DN ERROR".*"\]".*"'dn::execute_compose' can't find the docker-compose.yaml file 'docker-compose.unreachable.yaml'"
#  bats_print_run_env_variable
}

@test "docker command are passed to show and execute docker" {
  local DOCKER_CMD="build --no-cache --push"
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} --fail-fast -- ${DOCKER_CMD}
  assert_success
  assert_output --regexp .*"Skipping the execution of Docker command".*"docker compose -f dockerized-norlab-images/core-images/dependencies/docker-compose.dn-dependencies.build.yaml build --build-arg BUILDKIT_CONTEXT_KEEP_GIT_DIR=1 --no-cache --push".*"since the script is executed inside a docker container".*
#  bats_print_run_env_variable
}

@test "docker exit code propagation on pass › expect pass" {
  local DOCKER_CMD="version"
  mock_docker_command_exit_ok
  set +e
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                    --ci-test-force-runing-docker-cmd -- "$DOCKER_CMD"
  DOCKER_EXIT_CODE=$?

  set -e
  assert_equal "$DOCKER_EXIT_CODE" 0
}

@test "docker exit code propagation on faillure › expect pass" {
  local DOCKER_CMD="version"
  mock_docker_command_exit_error
  set +e
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                    --ci-test-force-runing-docker-cmd -- "$DOCKER_CMD"
  DOCKER_EXIT_CODE=$?

  set -e
  assert_equal "$DOCKER_EXIT_CODE" 1
}

@test "flags that set env variable" {
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                        --base-image dustynv/pytorch \
                        --os-name arbitratyName \
                        --tag-package 2.1 \
                        --tag-version r35.0.0 \
                        --docker-debug-logs \
                        --fail-fast

#                        --dockerized-norlab-version v0.3.0 \

  assert_not_empty "$IS_TEAMCITY_RUN"
#  assert_equal "${REPOSITORY_VERSION}" 'v0.3.0'
  assert_equal "${REPOSITORY_VERSION}" 'latest'
  assert_equal "${BASE_IMAGE}" 'dustynv/pytorch'
  assert_equal "${TAG_PACKAGE}" '2.1'
  assert_equal "${TAG_VERSION}" 'r35.0.0'
  assert_equal "${PROJECT_TAG}" 'arbitratyName-r35.0.0'
  assert_equal "${BUILDKIT_PROGRESS}" plain
  assert_equal "${DEPENDENCIES_BASE_IMAGE_TAG}" '2.1-r35.0.0'
  assert_equal "${DN_IMAGE_TAG}" 'DN-latest-2.1-r35.0.0'
}

@test "docker compose build conditional logic" {
#  skip "tmp dev"

  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                      --base-image dustynv/pytorch \
                      --os-name arbitratyName \
                      --tag-package 2.1 \
                      --tag-version r35.0.0 \
                      --docker-debug-logs \
                      --fail-fast \
                      -- build dependencies-core

  assert_output --regexp .*"Skipping the execution of Docker command".*"docker compose -f dockerized-norlab-images/core-images/dependencies/docker-compose.dn-dependencies.build.yaml build --build-arg BUILDKIT_CONTEXT_KEEP_GIT_DIR=1 dependencies-core".*"since the script is executed inside a docker container".*
}

@test "flag --help" {
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                                    --fail-fast \
                                    --help
  assert_success
  assert_output --regexp .*"dn::execute_compose <docker-compose.yaml> \[<optional flag>\] \[-- <any docker cmd\+arg>\]".*
}

@test "flag --buildx-bake" {
  local DOCKER_CMD="--load --push --builder jetson-nx-redleader-daemon"
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} --buildx-bake --fail-fast -- ${DOCKER_CMD}

  assert_success
  assert_output --regexp .*"docker buildx bake -f ".*"${DOCKER_CMD}"

#  bats_print_run_env_variable
}
