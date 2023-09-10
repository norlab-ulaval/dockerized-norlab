#!/bin/bash
#
# Convenient script for building all images specified in 'docker-compose.dn-dependencies.build.yaml'
#
# Usage:
#   $ bash dn_run_a_service.bash '<.env.build_matrix.*>' <service> [<optional command>]
#
# Arguments:
#   - <.env.build_matrix.*>  Dotenv build matrix file
#   - <service>              The service to run
#   - [<optional command>]   Any optional flag for docker compose run <service> [command]
#

clear

DOTENV_BUILD_MATRIX="${1:?' Missing the dotenv build matrix file mandatory argument'}"
shift # Remove argument value

if [[ $( basename "$(pwd)" ) = build_script ]]; then
    cd ../..
elif [[ $( basename "$(pwd)" ) = dockerized-norlab-scripts ]]; then
    cd ..
fi



# Notes;
#   - be advised that docker compose run command bypass the container_name field of the .yaml file so you can spin the same service multiple time and all container will have a unique name
bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash "${DOTENV_BUILD_MATRIX}" \
                                                            --fail-fast \
                                                            -- run --build --rm --no-deps "$@"

#--force-recreate
