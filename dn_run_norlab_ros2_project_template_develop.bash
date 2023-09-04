#!/bin/bash

# ››› Display and xhost ››› . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# (CRITICAL) ToDo: Check the Dusty-nv implementation for X11 forwarding (ref task NMO-183 Fix GUI display issue)

export DISPLAY=:0
#sudo xhost + # (Priority) todo:fixme!!
#   (ref task NMO-87 🩹→ Find a secure and permanent solution for the xhost "display not available" problem)
xhost +si:localuser:root
#  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .‹‹‹ Display and xhost ‹‹‹

source ./dn_which_architecture.bash

docker compose -f docker-compose.ros-project-template.jetson.run.yaml up --detach --wait

#CN=$(grep -A3 'develop:' docker-compose.ros-project-template.jetson.run.yaml | tail -n1); CN=${CN//*container_name: /}; echo "$CN"
#echo "${CN}"

if [[ -n $TEAMCITY_VERSION ]]; then
  # (NICE TO HAVE) ToDo: implement >> fetch container name from an .env file
  echo -e "${DS_MSG_EMPH_FORMAT}The container is running inside a TeamCity agent >> keep container detached${DS_MSG_END_FORMAT}"
else
  #  sudo docker exec -it ${CONTAINER_NAME} /ros_entrypoint.bash bash
  docker compose -f docker-compose.ros-project-template.jetson.run.yaml exec develop /ros2_entrypoint.bash bash
fi
