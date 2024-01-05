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
# Globals:
#   none
# =================================================================================================

# ToDo: refactor > use NS2T_PATH set somewhere
bash ./utilities/norlab-shell-script-tools/tests/bats_testing_tools/run_bats_tests_in_docker.bash "$@"
