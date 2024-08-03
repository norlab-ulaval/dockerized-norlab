#!/bin/bash
# =================================================================================================
# Execute 'dockerized-norlab' repo shell script tests via 'norlab-shell-script-tools' library
#
# Usage:
#   $ bash run_bats_core_test_in_n2st.bash ['<test-directory>[/<this-bats-test-file.bats>]' ['<image-distro>']]
#
# Arguments:
#   - ['<test-directory>']     The directory from which to start test, default to 'tests'
#   - ['<test-directory>/<this-bats-test-file.bats>']  A specific bats file to run, default will
#                                                      run all bats file in the test directory
#
# Globals: none
#
# =================================================================================================
PARAMS="$@"

if [[ -z $PARAMS ]]; then
  # Set to default bats tests directory if none specified
  PARAMS="tests/tests_bats/"
fi

function n2st::run_n2st_testsing_tools(){
  pushd "$(pwd)" >/dev/null || exit 1

# ....Project root logic.........................................................................
  DN_ROOT=$(git rev-parse --show-toplevel)

  # ....Load environment variables from file.......................................................
  cd "${DN_ROOT}" || exit 1
  set -o allexport
  source .env.dockerized-norlab-project
  source .env.dockerized-norlab-build-system
  set +o allexport

  # ....Execute N2ST run_bats_tests_in_docker.bash.................................................
  # shellcheck disable=SC2086
  bash "${N2ST_PATH:?err}/tests/bats_testing_tools/run_bats_tests_in_docker.bash" $PARAMS

  # ....Teardown...................................................................................
  popd >/dev/null || exit 1
  }

n2st::run_n2st_testsing_tools

