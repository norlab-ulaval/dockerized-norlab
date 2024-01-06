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
TESTED_FILE="dn_build_all.bash"
TESTED_FILE_PATH="dockerized-norlab-scripts/build_script"

setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  set -o allexport
  source .env.norlab-build-system
  set +o allexport


#  pwd >&3 && tree -L 2 -a -hug utilities/ >&3
#  printenv >&3
}

setup() {
#  cd "$TESTED_FILE_PATH" || exit
#  mock_docker_command_exit_ok

#  NBS_BUILD_MATRIX_CONFIG=tests/build_matrix_config && export NBS_BUILD_MATRIX_CONFIG
  export NBS_OVERRIDE_BUILD_MATRIX_MAIN='tests/.env.build_matrix.main.mock'

}

# ====Teardown======================================================================================

teardown() {
  bats_print_run_env_variable_on_error
}

#teardown_file() {
#    echo "executed once after finishing the last test"
#}

# ====Test casses===================================================================================


@test "running $TESTED_FILE from non 'root' › expect error" {
#  skip "tmp dev"

  cd "${BATS_DOCKER_WORKDIR}/dockerized-norlab-scripts/build_script"
  run bash ./$TESTED_FILE  --fail-fast
  assert_failure 1
  assert_output --partial "'$TESTED_FILE' script must be executed from the project root"
}

@test "running $TESTED_FILE from 'root' › expect pass" {
#  skip "tmp dev"

  cd "${BATS_DOCKER_WORKDIR}"

  assert_equal "$(basename $(pwd))" "dockerized-norlab"
  assert_file_exist .env.norlab-build-system
  assert_file_exist ${NBS_OVERRIDE_BUILD_MATRIX_MAIN}
  assert_file_exist ./build_matrix_config/test/.env.build_matrix.mock



  run source "./${TESTED_FILE_PATH}/$TESTED_FILE" "--fail-fast" -- "build --dry-run"
  assert_success
  refute_output --partial "'$TESTED_FILE' script must be executed from the project root"
}

@test "flag passed to 'dn_execute_compose_over_build_matrix.bash' › ok" {
#  skip "tmp dev"

  run source "./${TESTED_FILE_PATH}/$TESTED_FILE" \
                            --dockerized-norlab-version-build-matrix-override 'v8.8.8' \
                            --os-name-build-matrix-override 'l4t' \
                            --l4t-version-build-matrix-override 'r33.3.3' \
                            --fail-fast

  assert_success
  assert_output --regexp .*"docker compose -f".*"build".*

  assert_output --regexp .*"DN-v8.8.8-humble-ros-core-l4t-r33.3.3".*
  assert_output --regexp .*"DN-v8.8.8-humble-pytorch-l4t-r33.3.3".*

  refute_output --regexp .*"DN-v9.9.9-humble-ros-core-l4t-r11.1.1".*
  refute_output --regexp .*"DN-v9.9.9-humble-pytorch-l4t-r11.1.1".*

}


@test "env variable NBS_OVERRIDE_ADD_DOCKER_FLAG pass to script › ok" {
#  skip "tmp dev"

  local NBS_OVERRIDE_ADD_DOCKER_FLAG="--push --dry-run"
  run source ./${TESTED_FILE_PATH}/$TESTED_FILE \
                         --dockerized-norlab-version-build-matrix-override 'v8.8.8' \
                         --os-name-build-matrix-override 'l4t' \
                         --l4t-version-build-matrix-override 'r33.3.3' \
                         --fail-fast

  assert_output --regexp .*"docker compose -f".*"build ${NBS_OVERRIDE_ADD_DOCKER_FLAG}".*
#  bats_print_run_env_variable
}
