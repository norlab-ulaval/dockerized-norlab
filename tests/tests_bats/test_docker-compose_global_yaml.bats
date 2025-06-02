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

TESTED_FILE="docker-compose.global.yaml"
TESTED_FILE_PATH="./dockerized-norlab-images/core-images/global/"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  ## Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
  #pwd >&3 && tree -L 1 -a -hug >&3
  #printenv >&3
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

@test "check \"$TESTED_FILE\" service global-service-builder-config is set to multiarch › expect pass" {
#  cat $TESTED_FILE  >&3

  echo -e "\n\n
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[DN ERROR] -> This a '${TESTED_FILE}' misconfiguration error.

The MULTIARCH global-service-builder-config is muted. Its OK for developement.
Make sure the following lines look like this for PUSH TO CI BUILD:


  # # Note: DEV config for local build on native architecture.
  # # Keep MUTED for push to CI build. <--
  # global-service-builder-config:
  #   extends:
  #     service: global-service-builder-config-base-images
  #   build:
  #     platforms: !reset []

  # (CRITICAL) ToDo: Release config. Keep UNMUTED for push to CI build <--
  global-service-builder-config:
    extends:
      service: global-service-builder-config-base-images
    build:
      platforms:
        - linux/arm64
        - linux/amd64


!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
\n\n"

  # ....Test multiarch build config is NOT muted...................................................
  assert_file_exist $TESTED_FILE
  assert_file_contains "${TESTED_FILE}" "^  global-service-builder-config:$"
  assert_file_contains "${TESTED_FILE}" "^    extends:$"
  assert_file_contains "${TESTED_FILE}" "^      service: global-service-builder-config-base-images$"
  assert_file_contains "${TESTED_FILE}" "^    build:$"
  assert_file_contains "${TESTED_FILE}" "^      platforms:$"
  assert_file_contains "${TESTED_FILE}" "^        - linux/arm64$"
  assert_file_contains "${TESTED_FILE}" "^        - linux/amd64$"

  # ....Test native build config is muted..........................................................
  assert_file_contains "${TESTED_FILE}" "^.*#.*platforms: !reset \[\]$"
  assert_file_not_contains "${TESTED_FILE}" "^      platforms: !reset \[\]$"


}
