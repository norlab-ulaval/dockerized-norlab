#!/bin/bash
#
# Convenient script for building all images specified in 'docker-compose.dockerized-norlab.build.yaml'
#
# Usage:
#   $ bash dn_run_a_service.bash <service> [<optional command>]
#
# Arguments:
#   - <service>              The service to run
#   - [<optional command>]   Any optional flag for docker compose run <service> [command]
#

clear

if [[ $( basename "$(pwd)" ) = build_script ]]; then
    cd ../..
elif [[ $( basename "$(pwd)" ) = dockerized-norlab-scripts ]]; then
    cd ..
fi

# Notes;
#   - be advised that docker compose run command bypass the container_name field of the .yaml file so you can spin the same service multiple time and all container will have a unique name
bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                                                            --build-matrix-file-override .env.build_matrix.dev \
                                                            --fail-fast \
                                                            -- run --build --rm --no-deps "$@"

#--force-recreate
