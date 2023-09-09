#!/bin/bash

## ....Load helper function....................................................................................
#TMP_CWD=$(pwd)
#cd ./utilities/norlab-shell-script-tools/src/function_library
#source ./prompt_utilities.bash
#cd $TMP_CWD


# ====Begin===================================================================================================
bash ./utilities/norlab-shell-script-tools/tests/bats_testing_tools/run_bats_tests_in_docker.bash "$@"
