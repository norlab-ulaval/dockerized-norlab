#!/bin/bash
#
# Convenient script for spinning a specific service from the 'docker-compose.dn-dependencies.build.yaml'
#
# Usage:
#   $ bash dn_up_and_attach_to.bash '<.env.build_matrix.*>' <theService>
#
# Arguments:
#   - <theService>              The service to attach once all are up
#

DOTENV_BUILD_MATRIX="${1:?' Missing the dotenv build matrix file mandatory argument'}"
shift # Remove argument value

if [[ $( basename "$(pwd)" ) = build_script ]]; then
    cd ../..
elif [[ $( basename "$(pwd)" ) = dockerized-norlab-scripts ]]; then
    cd ..
fi



bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash "${DOTENV_BUILD_MATRIX}"\
                                                            --fail-fast \
                                                            -- up --build --detach --wait --no-deps "$1"


#CN=$(grep -A3 'project-develop:' ${THE_COMPOSE_FILE} | tail -n1); CN=${CN//*container_name: /}; echo "$CN"
#echo "${CN}"

if [[ -n $TEAMCITY_VERSION ]]; then
  # (NICE TO HAVE) ToDo: implement >> fetch container name from an .env file
  echo -e "${DS_MSG_EMPH_FORMAT}The container is running inside a TeamCity agent >> keep container detached${DS_MSG_END_FORMAT}"
else

  bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash "${DOTENV_BUILD_MATRIX}"\
                                                              --fail-fast \
                                                              -- exec "$1" bash
fi
