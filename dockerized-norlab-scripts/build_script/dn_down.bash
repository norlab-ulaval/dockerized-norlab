#!/bin/bash
#
# Convenient script for stopping services specified in 'docker-compose.dockerized-norlab.build.yaml'.
# All will be stopped if no argument are given.
#
# Usage:
#   $ bash dn_down.bash [<service>]
#
# Arguments:
#   - [services]              The service to stop
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
                                                            -- down "$@"

