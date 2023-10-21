#!/bin/bash

# ››› Display and xhost ››› . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# (Priority) ToDo: assessment >> display forwarding between remote device and local [Ubuntu+MacOs]
#   check: https://www.notion.so/redleader962/e4713bb868d949b1ab93351c564f66e7?pvs=4#a5e0797fb87f4f2aa1e3c628e9492a94
#   ref task NMO-183 Fix GUI display issue
# ToDo: validate >> check jetson-container implementation
#     from https://github.com/dusty-nv/jetson-containers/blob/master/run.sh



export DISPLAY=:0
#sudo xhost + # (Priority) todo:fixme!!
#   (ref task NMO-87 🩹→ Find a secure and permanent solution for the xhost "display not available" problem)
xhost +si:localuser:root
#  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .‹‹‹ Display and xhost ‹‹‹

source ./utilities/support_script/dn_which_architecture.bash

COMPOSE_FILE=Docker/dn-compose/docker-compose.ros-project-template.jetson.run.yaml

docker compose -f "${COMPOSE_FILE}" up --detach --wait

#CN=$(grep -A3 'project-develop:' "${COMPOSE_FILE}" | tail -n1); CN=${CN//*container_name: /}; echo "$CN"
#echo "${CN}"

if [[ -n $TEAMCITY_VERSION ]]; then
  # (NICE TO HAVE) ToDo: implement >> fetch container name from an .env file
  echo -e "${MSG_EMPH_FORMAT}The container is running inside a TeamCity agent >> keep container detached${MSG_END_FORMAT}"
else
  #  sudo docker exec -it ${DN_CONTAINER_NAME} /dn_ros2_entrypoint.bash bash
  docker compose -f "${COMPOSE_FILE}" exec develop /dn_ros2_entrypoint.bash bash
fi
