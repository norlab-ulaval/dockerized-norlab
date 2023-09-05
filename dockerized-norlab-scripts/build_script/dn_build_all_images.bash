#!/bin/bash

# ....Load helper function.........................................................................................
source ./utilities/support_script/dn_which_architecture.bash

TMP_CWD=$(pwd)
cd ./utilities/norlab-shell-script-tools/src/function_library
source ./prompt_utilities.bash
cd $TMP_CWD


# ====Begin========================================================================================================
# ....Build all images..............................................................................................
#docker compose -f <theComposeFile> build --no-cache

print_msg "Building 'docker-compose.ros-foxy-pytorch.jetson.build.yaml'"
docker compose -f Docker/dn-compose/docker-compose.dockerized-norlab.build.yaml build
print_msg_done "All build done"


