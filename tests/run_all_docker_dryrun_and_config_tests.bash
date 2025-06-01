#!/bin/bash
#
# Run all tests in directory
#
# Usage:
#   $ bash run_all_docker_dryrun_and_config_tests.bash
#
#

set -e            # exit on error
set -o nounset    # exit on unbound variable
set -o pipefail   # exit if errors within pipes

_PATH_TO_SCRIPT="$(realpath "$0")"
SCRIPT_DIR_PATH="$(dirname "${_PATH_TO_SCRIPT}")"
TEST_DIR="$SCRIPT_DIR_PATH/tests_docker_dryrun_and_config"

source "${SCRIPT_DIR_PATH}/../utilities/norlab-build-system/src/utility_scripts/nbs_run_all_test_and_dryrun_in_directory.bash" "${TEST_DIR}"
exit $?
