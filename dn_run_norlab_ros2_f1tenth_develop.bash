#!/bin/bash

# ››› Display and xhost ››› . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
export DISPLAY=:0
#sudo xhost + # (Priority) todo:fixme!! (ref task NLSAR-189)
xhost +si:localuser:root
#  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .‹‹‹ Display and xhost ‹‹‹

# ....Run norlab-ros2-humble-develop:arm64-l4t-r35.2.1..............................................................

docker compose -f docker-compose.ros2-f1tenth.jetson.run.yaml up --detach --wait

#CN=$(grep -A3 'develop:' docker-compose.ros2.jetson.run.yaml | tail -n1); CN=${CN//*container_name: /}; echo "$CN"
#echo "${CN}"

if [[ -n $TEAMCITY_VERSION ]]; then
  # (NICE TO HAVE) ToDo: implement >> fetch container name from an .env file
  echo -e "${DS_MSG_EMPH_FORMAT}The container is running inside a TeamCity agent >> keep container detached${DS_MSG_END_FORMAT}"
else
  #  sudo docker exec -it ${CONTAINER_NAME} /ros_entrypoint.bash bash
  docker compose -f docker-compose.ros2-f1tenth.jetson.run.yaml exec develop /ros2_entrypoint.bash bash
fi
