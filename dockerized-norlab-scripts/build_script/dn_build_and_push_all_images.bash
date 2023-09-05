#!/bin/bash

source ./utilities/support_script/dn_which_architecture.bash

set -o allexport; source .env.prompt; set +o allexport

# ....Build all images..............................................................................................
#docker compose -f <theComposeFile> build --no-cache

echo -e "${MSG_BASE} Building 'docker-compose.ros-foxy-pytorch.jetson.build.yaml'"
docker compose -f Docker/dn-compose/docker-compose.dockerized-norlab.build.yaml build --push
echo -e "${MSG_DONE} Build and push done"

echo -e "${MSG_DONE} All build and push done"
