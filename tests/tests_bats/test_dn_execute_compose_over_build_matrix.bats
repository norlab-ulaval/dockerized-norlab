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

  export NBS_OVERRIDE_BUILD_MATRIX_MAIN=build_matrix_config/test/.env.build_matrix.main.mock
  export BUILD_MATRIX_CONFIG_FILE=build_matrix_config/test/.env.build_matrix.mock

#  pwd >&3 && tree -L 2 -a -hug >&3
#  printenv >&3
}

setup() {
#  cd "${DN_PATH:?err}"
  source "import_dockerized_norlab_tools.bash" || exit 1

  cd "$BATS_DOCKER_WORKDIR" || exit
}

# ====Teardown======================================================================================

#teardown() {
#  bats_print_run_env_variable_on_error
##  bats_print_run_env_variable
#}

#teardown_file() {
#    echo "executed once after finishing the last test"
#}

# ====Test casses===================================================================================


@test "executing $TESTED_FILE from bad cwd › expect fail" {
#  skip "tmp dev"

  cd "${BATS_DOCKER_WORKDIR}/dockerized-norlab-scripts/"
  run bash ./build_script/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE}

  assert_failure 1
  assert_output --partial "'$TESTED_FILE' script must be executed from the project root"
}


@test "executing $TESTED_FILE from ok cwd › expect pass" {
#  skip "tmp dev"

  cd "${BATS_DOCKER_WORKDIR}"
  run bash -c "bash ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE}"
  assert_success
  refute_output  --partial "No such file or directory"
#  bats_print_run_env_variable
}

@test "missing dotenv build matrix file mandatory argument› expect pass" {
#  skip "tmp dev"

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
#  skip "tmp dev"

  local DOCKER_CMD="build --no-cache --push"
  run bash -c "bash ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} -- ${DOCKER_CMD}"
  assert_success
  assert_output --regexp .*"Skipping the execution of Docker command".*
  assert_output --regexp .*"docker compose -f ".*"${DOCKER_CMD}".*
  assert_output --regexp .*"with command".*"${DOCKER_CMD}".*
}

@test "dotenv build matrix file argument › ok" {
#  skip "tmp dev"

  assert_equal "$(basename $(pwd))" "dockerized-norlab"
  assert_file_exist .env.dockerized-norlab-build-system
  assert_file_exist "${BUILD_MATRIX_CONFIG_FILE}"

  local DOCKER_CMD="version"
  local _CI_TEST=true
  set +e
  mock_docker_command_exit_ok
  run source ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} \
                                              --ci-test-force-runing-docker-cmd \
                                              -- "$DOCKER_CMD"
  set -e
  assert_success
  assert_output --regexp .*"NBS_MATRIX_L4T_SUPPORTED_VERSIONS\=\(r11.1.1 r22.2.2\)".*
#  bats_print_run_env_variable
}

@test "dotenv build matrix source ordering › ok" {
#  skip "tmp dev"

  assert_equal "$(basename $(pwd))" "dockerized-norlab"
  assert_file_exist .env.dockerized-norlab-build-system
  assert_file_exist .env.build_matrix.main
  assert_file_exist "${NBS_OVERRIDE_BUILD_MATRIX_MAIN}"
  assert_file_exist "${BUILD_MATRIX_CONFIG_FILE}"

  local DOCKER_CMD="version"
  local _CI_TEST=true
  set +e
  mock_docker_command_exit_ok
  run source ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} \
                                              --ci-test-force-runing-docker-cmd \
                                              -- "$DOCKER_CMD"
  set -e
  assert_success
  assert_output --regexp "\[DN-build-system\]".*"Loading main build matrix".*".env.build_matrix.main"
  assert_output --regexp "\[DN-build-system\]".*"Loading main build matrix override".*"${NBS_OVERRIDE_BUILD_MATRIX_MAIN}"
  assert_output --regexp "\[DN-build-system\]".*"Loading build matrix".*"${BUILD_MATRIX_CONFIG_FILE}"

#  bats_print_run_env_variable
}

@test "docker exit code propagation on pass › expect pass" {

  local DOCKER_CMD="version"
  local _CI_TEST=true
  set +e

  mock_docker_command_exit_ok
  run source ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} \
                                          --ci-test-force-runing-docker-cmd \
                                          -- "$DOCKER_CMD"
  set -e
  assert_success
  assert_output --regexp .*"Pass".*"DN-hot-humble-ros-core-l4t-r11.1.1".*
  assert_output --regexp .*"Pass".*"DN-hot-humble-pytorch-l4t-r11.1.1".*
  assert_output --regexp .*"Pass".*"DN-hot-humble-pytorch-l4t-r22.2.2".*
  assert_output --regexp .*"Pass".*"DN-hot-humble-pytorch-l4t-r22.2.2".*
  assert_output --regexp .*"Pass".*"DN-v0.3.0-humble-ros-core-l4t-r11.1.1".*
  assert_output --regexp .*"Pass".*"DN-v0.3.0-humble-pytorch-l4t-r11.1.1".*
  assert_output --regexp .*"Pass".*"DN-v0.3.0-humble-pytorch-l4t-r22.2.2".*
  assert_output --regexp .*"Pass".*"DN-v0.3.0-humble-pytorch-l4t-r22.2.2".*
#  bats_print_run_env_variable
}

@test "docker exit code propagation on faillure › expect pass" {

  local DOCKER_CMD="version"
  local _CI_TEST=true
  fake_IS_TEAMCITY_RUN
  set +e

  mock_docker_command_exit_error
  run source ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} \
                                              --ci-test-force-runing-docker-cmd \
                                                -- "$DOCKER_CMD"
  set -e
  assert_failure
  assert_output --regexp .*"Fail".*"DN-hot-humble-ros-core-l4t-r11.1.1".*
  assert_output --regexp .*"Fail".*"DN-hot-humble-pytorch-l4t-r11.1.1".*
  assert_output --regexp .*"Fail".*"DN-hot-humble-pytorch-l4t-r22.2.2".*
  assert_output --regexp .*"Fail".*"DN-hot-humble-pytorch-l4t-r22.2.2".*
  assert_output --regexp .*"Fail".*"DN-v0.3.0-humble-ros-core-l4t-r11.1.1".*
  assert_output --regexp .*"Fail".*"DN-v0.3.0-humble-pytorch-l4t-r11.1.1".*
  assert_output --regexp .*"Fail".*"DN-v0.3.0-humble-pytorch-l4t-r22.2.2".*
  assert_output --regexp .*"Fail".*"DN-v0.3.0-humble-pytorch-l4t-r22.2.2".*
}

@test "docker exit code propagation on faillure › expect pass (TeamCity casses)" {

  local DOCKER_CMD="version"
  local _CI_TEST=true
  fake_IS_TEAMCITY_RUN
  set +e

  mock_docker_command_exit_error
  run source ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} \
                                        --ci-test-force-runing-docker-cmd \
                                        -- "$DOCKER_CMD"
  set -e
  assert_failure
  assert_output --regexp "\#\#teamcity\[buildProblem description='BUILD FAIL with docker exit code: 1'\]"
#  bats_print_run_env_variable
}


@test "flags that set env variable" {
#  skip "tmp dev"

  set +e
  mock_docker_command_exit_ok
  run bash ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} \
                                      --dockerized-norlab-version-build-matrix-override 'v0.2.0' \
                                      --os-name-build-matrix-override 'l4t' \
                                      --l4t-version-build-matrix-override 'r33.3.3'
  set -e
  assert_output --regexp .*"Pass".*"DN-v0.2.0-humble-ros-core-l4t-r33.3.3".*
  assert_output --regexp .*"Pass".*"DN-v0.2.0-humble-pytorch-l4t-r33.3.3".*

  refute_output --regexp .*"Pass".*"DN-v0.3.0-humble-ros-core-l4t-r11.1.1".*
  refute_output --regexp .*"Pass".*"DN-v0.3.0-humble-pytorch-l4t-r11.1.1".*
}

@test "--force-push 'latest' tag sanity check ok" {
  if [[ $(git symbolic-ref -q --short HEAD) == main ]]; then
    skip "Curent checkout branch is 'main' which invalidate this test logic"
  fi

#  set +e
  mock_docker_command_exit_ok
  run bash ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} \
                                      --dockerized-norlab-version-build-matrix-override 'latest' \
                                      --os-name-build-matrix-override 'l4t' \
                                      --l4t-version-build-matrix-override 'r33.3.3'
#  set -e
  assert_output --regexp .*"\[".*"DN-build-system error".*"\]".*"The DN 'latest' tag was set but the current checkout branch is not the 'main' branch."
}

@test "--force-push 'bleeding' tag sanity check ok" {
  if [[ $(git symbolic-ref -q --short HEAD) == dev ]]; then
    skip "Curent checkout branch is 'dev' which invalidate this test logic"
  fi

#  set +e
  mock_docker_command_exit_ok
  run bash ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} \
                                      --dockerized-norlab-version-build-matrix-override 'bleeding' \
                                      --os-name-build-matrix-override 'l4t' \
                                      --l4t-version-build-matrix-override 'r33.3.3'
#  set -e
  assert_output --regexp .*"\[".*"DN-build-system error".*"\]".*"The DN 'bleeding' tag was set but the current checkout branch is not the 'dev' branch."
}

@test "flag --force-push is passed to dn_execute_compose.bash" {
#  skip "tmp dev"

  local _CI_TEST=true
  mock_docker_command_config_services
#  run bash -c "bash ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} --force-push -- build"
  run source ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} \
                              --force-push \
                              --ci-test-force-runing-docker-cmd \
                              -- build

  assert_success

  assert_output --regexp .*"\[".*"DN-build-system done".*"\]".*"Command".*"docker compose -f dockerized-norlab-images/core-images/dependencies/docker-compose.dn-dependencies.build.yaml build --build-arg BUILDKIT_CONTEXT_KEEP_GIT_DIR=1 mock-service-one".*"completed successfully and exited docker.".*"\[".*"DN-build-system".*"\]".*"Force push mock-service-one image to docker registry".*"\[".*"DN-build-system".*"\]".*"\[".*"DN-build-system done".*"\]".*"Command".*"docker compose -f dockerized-norlab-images/core-images/dependencies/docker-compose.dn-dependencies.build.yaml push mock-service-one".*"completed successfully and exited docker.".*"\[".*"DN-build-system done".*"\]".*"Command".*"docker compose -f dockerized-norlab-images/core-images/dependencies/docker-compose.dn-dependencies.build.yaml build --build-arg BUILDKIT_CONTEXT_KEEP_GIT_DIR=1 mock-service-two".*"completed successfully and exited docker.".*"\[".*"DN-build-system".*"\]".*"Force push mock-service-two image to docker registry".*"\[".*"DN-build-system".*"\]".*"\[".*"DN-build-system done".*"\]".*"Command".*"docker compose -f dockerized-norlab-images/core-images/dependencies/docker-compose.dn-dependencies.build.yaml push mock-service-two".*"completed successfully and exited docker.".*
}

@test "flag --help" {
#  skip "tmp dev"

  run bash ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} \
                                                                 --help
  assert_success
  assert_output --regexp .*"${TESTED_FILE} '<.env.build_matrix.*>' \[<optional flag>\] \[-- <any docker cmd\+arg>\]".*
}

@test "flag --buildx-bake" {
#  skip "tmp dev"

  local DOCKER_CMD="--load --push --builder jetson-nx-redleader-daemon"
  run bash -c "bash ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE} --buildx-bake -- ${DOCKER_CMD}"

  assert_success
  assert_output --regexp .*"docker buildx bake -f ".*"${DOCKER_CMD}"
#  bats_print_run_env_variable
}


@test "repository version checkout" {

  cd "${BATS_DOCKER_WORKDIR}"
  run bash -c "bash ./${TESTED_FILE_PATH}/$TESTED_FILE ${BUILD_MATRIX_CONFIG_FILE}" \
                        --dockerized-norlab-version-build-matrix-override 'v0.2.0' \
                        --os-name-build-matrix-override 'l4t' \
                        --l4t-version-build-matrix-override 'r33.3.3' \
                        --fail-fast

  assert_output --regexp .*"\[".*"DN-build-system".*"\]".*"Git fetch tag list".*"v0.2.0".*"v0.3.0".*"\[".*"DN-build-system".*"\]".*"\[".*"DN-build-system warning".*"\]".*"Bats test run › skip \"Execute git checkout\"".*"\[".*"DN-build-system".*"\]".*"Repository checkout".*

}
