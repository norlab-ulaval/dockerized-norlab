#!/bin/bash
# =================================================================================================
# Run all tests in directory "tests_dryrun_and_tests_scripts/"
#
# Usage:
#   $ bash run_all_dryrun_and_tests_scripts.bash
#
# =================================================================================================

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/import_dockerized_norlab_tools.bash" || exit 1
test_dir="${DN_PATH:?err}/tests/tests_dryrun_and_tests_scripts"

# ....Begin........................................................................................
source "${NBS_PATH:?err}/src/utility_scripts/nbs_run_all_test_and_dryrun_in_directory.bash" "${test_dir}"
exit $?


