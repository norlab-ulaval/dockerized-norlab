#!/bin/bash

source ./dn_which_architecture.bash

set -o allexport; source .env.prompt; set +o allexport

# ....Build all images..............................................................................................
#docker compose -f <theComposeFile> build --no-cache

#echo -e "${DS_MSG_BASE} Building 'docker-compose.ros-humble-pytorch.jetson.build.yaml'"
#docker compose -f docker-compose.ros-humble-pytorch.jetson.build.yaml build --push
#echo -e "${DS_MSG_DONE} Build and push done"

echo -e "${DS_MSG_BASE} Building 'docker-compose.ros-foxy-pytorch.jetson.build.yaml'"
docker compose -f docker-compose.ros-foxy-pytorch.jetson.build.yaml build --push
echo -e "${DS_MSG_DONE} Build and push done"

echo -e "${DS_MSG_DONE} All build and push done"
