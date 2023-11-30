#!/bin/bash

## ....Load helper function..........................................................................
#TMP_CWD=$(pwd)
## ToDo: refactor > use NS2T_PATH set somewhere
#cd ./utilities/norlab-shell-script-tools/src/function_library
#source ./prompt_utilities.bash
#cd $TMP_CWD


# ====Begin=========================================================================================
# ToDo: refactor > use NS2T_PATH set somewhere
bash ./utilities/norlab-shell-script-tools/tests/bats_testing_tools/run_bats_tests_in_docker.bash "$@"
