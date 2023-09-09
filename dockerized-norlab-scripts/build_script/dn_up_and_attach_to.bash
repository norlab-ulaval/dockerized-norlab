#!/bin/bash
#
# Convenient script for spinning a specific service from the 'docker-compose.dockerized-norlab.build.yaml'
#
# Usage:
#   $ bash dn_up_and_attach_to.bash <theService>
#
# Arguments:
#   - <theService>              The service to attach once all are up
#


if [[ $( basename "$(pwd)" ) = build_script ]]; then
    cd ../..
elif [[ $( basename "$(pwd)" ) = dockerized-norlab-scripts ]]; then
    cd ..
fi


# ToDo: on task end >> refactor out the `--build-matrix-file-override .env.build_matrix.dev` line
bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                                                            --build-matrix-file-override .env.build_matrix.dev \
                                                            --fail-fast \
                                                            -- up --build --detach --wait --no-deps "$1"


#CN=$(grep -A3 'develop:' ${THE_COMPOSE_FILE} | tail -n1); CN=${CN//*container_name: /}; echo "$CN"
#echo "${CN}"

if [[ -n $TEAMCITY_VERSION ]]; then
  # (NICE TO HAVE) ToDo: implement >> fetch container name from an .env file
  echo -e "${DS_MSG_EMPH_FORMAT}The container is running inside a TeamCity agent >> keep container detached${DS_MSG_END_FORMAT}"
else
  # ToDo: on task end >> refactor out the `--build-matrix-file-override .env.build_matrix.dev` line
  bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                                                              --build-matrix-file-override .env.build_matrix.dev \
                                                              --fail-fast \
                                                              -- exec "$1" bash
fi
