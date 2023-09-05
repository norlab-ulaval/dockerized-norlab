#!/bin/bash

source ./utilities/support_script/dn_which_architecture.bash

docker compose -f Docker/dn-compose/docker-compose.ros-project-template.jetson.run.yaml down

# docker exec -it IamSNOW-NX-TEST bash



