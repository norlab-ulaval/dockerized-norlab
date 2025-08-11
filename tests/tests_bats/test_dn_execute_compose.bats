#!/usr/bin/env bats
# =================================================================================================
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
# =================================================================================================

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

# ====Setup========================================================================================
TESTED_FILE="dn_execute_compose.bash"
TESTED_FILE_PATH="dockerized-norlab-scripts/build_script"

TEST_DOCKER_COMPOSE_FILE_PATH=dockerized-norlab-images/core-images/base-images/l4t-images
TEST_DOCKER_COMPOSE_FILE="${TEST_DOCKER_COMPOSE_FILE_PATH}/docker-compose.l4t-squash.build.yaml"
#TEST_DOCKER_COMPOSE_FILE_PATH=dockerized-norlab-images/core-images/base-images/ros2-install
#TEST_DOCKER_COMPOSE_FILE="${TEST_DOCKER_COMPOSE_FILE_PATH}/docker-compose.ros2.build.yaml"

setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
#  pwd >&3 && tree -L 2 -a -hug utilities/ >&3
#  printenv >&3


}

setup() {
  source "import_dockerized_norlab_tools.bash" || exit 1

  set +o allexport
  source "${BATS_DOCKER_WORKDIR}/build_matrix_config/test/.env.build_matrix.main.mock"
  source "${BATS_DOCKER_WORKDIR}/build_matrix_config/test/.env.build_matrix.mock"
  set -o allexport

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

  assert_equal "${DN_IMPORTED}" "true"

  source ./build_script/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                          --dockerized-norlab-version hot \
                          --base-image dustynv/pytorch \
                          --os-name l4t \
                          --ros2 foxy-ros-base \
                          --base-img-tag-prefix 2.1 \
                          --tag-os-version r35.0.0

  assert_failure 1
  assert_output --regexp .*"\[".*"DN-build-system error".*"\]".*"'dn::execute_compose' function must be executed from the project root!"
}

@test "sourcing $TESTED_FILE from ok cwd › expect pass" {
  assert_equal "${DN_IMPORTED}" "true"

  mock_docker_command_config_services
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                          --dockerized-norlab-version hot \
                          --base-image dustynv/pytorch \
                          --os-name l4t \
                          --ros2 foxy-ros-base \
                          --base-img-tag-prefix 2.1 \
                          --tag-os-version r35.0.0

  assert_success
  refute_output --regexp .*"\[".*"DN-build-system error".*"\]".*"'dn::execute_compose' function must be executed from the project root!"
#  bats_print_run_env_variable
}

@test "missing docker-compose.yaml file mandatory argument › expect fail" {
  assert_equal "${DN_IMPORTED}" "true"

  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose

  assert_output --partial 'Missing the docker-compose.yaml file mandatory argument'

  run dn::execute_compose "docker-compose.unreachable.yaml" \
                            --dockerized-norlab-version hot \
                            --base-image dustynv/pytorch \
                            --os-name l4t \
                            --ros2 foxy-ros-base \
                            --base-img-tag-prefix 2.1 \
                            --tag-os-version r35.0.0
  assert_failure
  assert_output --regexp .*"\[".*"DN-build-system error".*"\]".*"'dn::execute_compose' can't find the docker-compose.yaml file 'docker-compose.unreachable.yaml'"
#  bats_print_run_env_variable
}

@test "docker command are passed to show and execute docker" {
  assert_equal "${DN_IMPORTED}" "true"

  local DOCKER_CMD="build --no-cache --push"
  mock_docker_command_config_services
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE}  \
                          --dockerized-norlab-version hot \
                          --base-image dustynv/pytorch \
                          --os-name l4t \
                          --ros2 foxy-ros-base \
                          --base-img-tag-prefix 2.1 \
                          --tag-os-version r35.0.0 \
                          -- ${DOCKER_CMD}
  assert_success
  assert_output --regexp .*"Skipping the execution of Docker command".*"docker compose -f dockerized-norlab-images/core-images/base-images/l4t-images/docker-compose.l4t-squash.build.yaml".*" build --build-arg BUILDKIT_CONTEXT_KEEP_GIT_DIR=1 --no-cache --push".*"since the script is executed inside a docker container".*
#  bats_print_run_env_variable
}

@test "docker-compose.global.yaml is merged to main compose file" {
  assert_equal "${DN_IMPORTED}" "true"

  COMPOSE_FILE_GLOBAL="dockerized-norlab-images/core-images/global/docker-compose.global.yaml"

  mock_docker_command_config_services
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE}  \
                          --dockerized-norlab-version hot \
                          --base-image dustynv/pytorch \
                          --os-name l4t \
                          --ros2 foxy-ros-base \
                          --base-img-tag-prefix 2.1 \
                          --tag-os-version r35.0.0 \
                          --ci-test-force-runing-docker-cmd \
                          -- build

  assert_success
  assert_output --regexp .*"\[".*"DN-build-system".*"\]".*"Command".*"docker compose -f dockerized-norlab-images/core-images/base-images/l4t-images/docker-compose.l4t-squash.build.yaml -f ${COMPOSE_FILE_GLOBAL} build --build-arg BUILDKIT_CONTEXT_KEEP_GIT_DIR=1".*"completed successfully and exited docker."

}

@test "docker exit code propagation on pass › expect pass" {
  assert_equal "${DN_IMPORTED}" "true"

  local DOCKER_CMD="version"
  set +e
  mock_docker_command_config_services
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                    --dockerized-norlab-version hot \
                    --base-image dustynv/l4t-pytorch \
                    --os-name l4t \
                    --ros2 foxy-ros-base \
                    --base-img-tag-prefix 2.1 \
                    --tag-os-version r35.0.0 \
                    --ci-test-force-runing-docker-cmd -- ${DOCKER_CMD}
  DOCKER_EXIT_CODE=$?

  set -e
  assert_equal "$DOCKER_EXIT_CODE" 0
}

@test "docker exit code propagation on faillure › expect pass" {
  assert_equal "${DN_IMPORTED}" "true"

  local DOCKER_CMD="version"
  mock_docker_command_exit_error
  set +e
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                    --dockerized-norlab-version hot \
                    --base-image dustynv/pytorch \
                    --os-name l4t \
                    --ros2 foxy-ros-base \
                    --base-img-tag-prefix 2.1 \
                    --tag-os-version r35.0.0 \
                    --ci-test-force-runing-docker-cmd -- "$DOCKER_CMD"
  DOCKER_EXIT_CODE=$?

  set -e
  assert_equal "$DOCKER_EXIT_CODE" 1
}

@test "Variable are exported to calling script › expect pass" {
  assert_equal "${DN_IMPORTED}" "true"

  unset BUILDKIT_PROGRESS
  #assert_empty "$BUILDKIT_PROGRESS"
  assert_empty "$REPOSITORY_VERSION"
  assert_empty "$BASE_IMAGE"
  assert_empty "$OS_NAME"
  assert_empty "$BASE_IMG_TAG_PREFIX"
  assert_empty "$TAG_OS_VERSION"
  assert_empty "$DEPENDENCIES_BASE_IMAGE"
  assert_empty "$DEPENDENCIES_BASE_IMAGE_TAG"
  assert_empty "$DN_IMAGE_TAG"
  assert_empty "$PROJECT_TAG"

  local DOCKER_CMD="version"
  mock_docker_command_exit_ok
  set +e
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                    --dockerized-norlab-version hot \
                    --base-image dustynv/pytorch \
                    --os-name l4t \
                    --ros2 foxy-ros-base \
                    --base-img-tag-prefix 2.1 \
                    --tag-os-version r35.0.0 \
                    --docker-debug-logs \
                    --ci-test-force-runing-docker-cmd -- "$DOCKER_CMD"

  DOCKER_EXIT_CODE=$?
  set -e

  assert_not_empty "$BUILDKIT_PROGRESS"
  assert_not_empty "$REPOSITORY_VERSION"
  assert_not_empty "$BASE_IMAGE"
  assert_not_empty "$OS_NAME"
  assert_not_empty "$BASE_IMG_TAG_PREFIX"
  assert_not_empty "$TAG_OS_VERSION"
  assert_not_empty "$DEPENDENCIES_BASE_IMAGE"
  assert_not_empty "$DEPENDENCIES_BASE_IMAGE_TAG"
  assert_not_empty "$DN_IMAGE_TAG"
  assert_not_empty "$PROJECT_TAG"

  assert_equal "$BUILDKIT_PROGRESS" "plain"
  assert_equal "$REPOSITORY_VERSION" "hot"
  assert_equal "$BASE_IMAGE" "dustynv/pytorch"
  assert_equal "$OS_NAME" "l4t"
  assert_equal "$BASE_IMG_TAG_PREFIX" "2.1"
  assert_equal "$TAG_OS_VERSION" "r35.0.0"
  assert_equal "$DEPENDENCIES_BASE_IMAGE" "dustynv/pytorch"
  assert_equal "$DEPENDENCIES_BASE_IMAGE_TAG" "2.1-r35.0.0"
  assert_equal "$DN_IMAGE_TAG" "DN-hot-foxy-base-pytorch-2.1-r35.0.0"
  assert_equal "$PROJECT_TAG" "l4t-r35.0.0"
}

@test "flags that set env variable" {
  assert_equal "${DN_IMPORTED}" "true"

  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                        --dockerized-norlab-version v0.3.0 \
                        --base-image dustynv/pytorch \
                        --os-name l4t \
                        --ros2 foxy-ros-base \
                        --base-img-tag-prefix 2.1 \
                        --tag-os-version r35.0.0 \
                        --docker-debug-logs

  assert_output --regexp .*"\[".*"DN-build-system".*"\]".*"IS_TEAMCITY_RUN=".*
  assert_output --regexp .*"\[".*"DN-build-system".*"\]".*"Environment variables set for".*"compose".*"REPOSITORY_VERSION=".*"v0.3.0".*"DEPENDENCIES_BASE_IMAGE=".*"dustynv/pytorch".*"TAG_OS_VERSION=".*"r35.0.0".*"DEPENDENCIES_BASE_IMAGE_TAG=".*"2.1-r35.0.0".*"DN_IMAGE_TAG=".*"DN-v0.3.0-foxy-base-pytorch-2.1-r35.0.0".*"PROJECT_TAG=".*"l4t-r35.0.0"
}

@test "docker compose build conditional logic" {
  assert_equal "${DN_IMPORTED}" "true"

  mock_docker_command_config_services
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                      --dockerized-norlab-version hot \
                      --base-image dustynv/pytorch \
                      --os-name l4t \
                      --ros2 foxy-ros-base \
                      --base-img-tag-prefix 2.1 \
                      --tag-os-version r35.0.0 \
                      --docker-debug-logs \
                      -- build base-image-tester

  assert_output --regexp .*"Skipping the execution of Docker command".*"docker compose -f dockerized-norlab-images/core-images/base-images/l4t-images/docker-compose.l4t-squash.build.yaml".*" build --build-arg BUILDKIT_CONTEXT_KEEP_GIT_DIR=1 base-image-tester".*"since the script is executed inside a docker container".*
}

@test "flag --force-push" {
  assert_equal "${DN_IMPORTED}" "true"

  mock_docker_command_config_services
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                      --dockerized-norlab-version hot \
                      --base-image dustynv/pytorch \
                      --os-name l4t \
                      --ros2 foxy-ros-base \
                      --base-img-tag-prefix 2.1 \
                      --tag-os-version r35.0.0 \
                      --force-push \
                      -- build

  assert_output --regexp .*"Skipping the execution of Docker command".*"docker compose -f dockerized-norlab-images/core-images/base-images/l4t-images/docker-compose.l4t-squash.build.yaml".*" build --build-arg BUILDKIT_CONTEXT_KEEP_GIT_DIR=1 mock-service-one".*"since the script is executed inside a docker container".*"Force push mock-service-one image to docker registry".*"Skipping the execution of Docker command".*"docker compose -f dockerized-norlab-images/core-images/base-images/l4t-images/docker-compose.l4t-squash.build.yaml".*" push mock-service-one".*"since the script is executed inside a docker container".*"Skipping the execution of Docker command".*"docker compose -f dockerized-norlab-images/core-images/base-images/l4t-images/docker-compose.l4t-squash.build.yaml".*" build --build-arg BUILDKIT_CONTEXT_KEEP_GIT_DIR=1 mock-service-two".*"since the script is executed inside a docker container".*"Force push mock-service-two image to docker registry".*"Skipping the execution of Docker command".*"docker compose -f dockerized-norlab-images/core-images/base-images/l4t-images/docker-compose.l4t-squash.build.yaml".*" push mock-service-two".*"since the script is executed inside a docker container".*
}

@test "flag --help" {
  assert_equal "${DN_IMPORTED}" "true"

  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} --help
  assert_success
  assert_output --regexp .*"dn::execute_compose <docker-compose.yaml> \[<optional flag>\] \[-- <any docker cmd\+arg>\]".*
}

@test "flag --buildx-bake" {
  assert_equal "${DN_IMPORTED}" "true"

  local DOCKER_CMD="--load --push --builder jetson-nx-redleader-daemon"
  mock_docker_command_config_services
  source ./${TESTED_FILE_PATH}/$TESTED_FILE
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                          --dockerized-norlab-version hot \
                          --base-image dustynv/pytorch \
                          --os-name l4t \
                          --ros2 foxy-ros-base \
                          --base-img-tag-prefix 2.1 \
                          --tag-os-version r35.0.0 \
                          --buildx-bake -- ${DOCKER_CMD}

  assert_success
  assert_output --regexp .*"docker buildx bake -f ".*"${DOCKER_CMD}"
}

# ....base image › l4t-images......................................................................

@test "base image › l4t-images | dn_callback_execute_compose_post.bash › OK" {
  assert_equal "${DN_IMPORTED}" "true"

  mock_docker_command_config_services
  source ./${TESTED_FILE_PATH}/$TESTED_FILE

  # . . Case: push manifest. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . ..
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                      --dockerized-norlab-version hot \
                      --base-image dustynv/pytorch \
                      --os-name l4t \
                      --ros2 foxy-ros-base \
                      --base-img-tag-prefix 2.1 \
                      --tag-os-version r35.0.0 \
                      -- build --push

  assert_output --regexp .*"Preparing docker manifeste for multiarch image".*"Multiarch image pushed to docker registry".*

  # . . Case: no push manifest. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                        --dockerized-norlab-version hot \
                        --base-image dustynv/pytorch \
                        --os-name l4t \
                        --ros2 foxy-ros-base \
                        --base-img-tag-prefix 2.1 \
                        --tag-os-version r35.0.0 \
                        -- build --push --dry-run

  assert_output --regexp .*"Skip pushing dockerized-norlab-base-image multi-arch image".*

  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                        --dockerized-norlab-version hot \
                        --base-image dustynv/pytorch \
                        --os-name l4t \
                        --ros2 foxy-ros-base \
                        --base-img-tag-prefix 2.1 \
                        --tag-os-version r35.0.0 \
                        -- build

  assert_output --regexp .*"Skip pushing dockerized-norlab-base-image multi-arch image".*

  run dn::execute_compose ${TEST_DOCKER_COMPOSE_FILE} \
                        --dockerized-norlab-version hot \
                        --base-image dustynv/pytorch \
                        --os-name l4t \
                        --ros2 foxy-ros-base \
                        --base-img-tag-prefix 2.1 \
                        --tag-os-version r35.0.0 \
                        -- build --dry-run

  assert_output --regexp .*"Skip pushing dockerized-norlab-base-image multi-arch image".*
}
