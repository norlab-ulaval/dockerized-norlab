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

if [[ $( basename $(pwd) ) = build_script ]]; then
    cd ../..
elif [[ $( basename $(pwd) ) = dockerized-norlab-scripts ]]; then
    cd ..
fi

bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                                                            --build-matrix-file-override .env.build_matrix.dev \
                                                            --fail-fast \
                                                            -- run --build --rm "$@"
#--force-recreate
