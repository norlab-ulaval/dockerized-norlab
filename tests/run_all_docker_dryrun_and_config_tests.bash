#!/bin/bash
# =================================================================================================
# Run all tests in directory
#
# Usage:
#   $ bash run_all_docker_dryrun_and_config_tests.bash
#
# =================================================================================================

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/import_dockerized_norlab_tools.bash" || exit 1
test_dir="${DN_PATH:?err}/tests/tests_docker_dryrun_and_config"

# ....Begin........................................................................................
source "${NBS_PATH:?err}/src/utility_scripts/nbs_run_all_test_and_dryrun_in_directory.bash" "${test_dir}"



