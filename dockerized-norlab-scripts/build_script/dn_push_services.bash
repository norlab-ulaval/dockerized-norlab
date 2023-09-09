#!/bin/bash
#
# Convenient script for pushing all images specified in 'docker-compose.dockerized-norlab.build.yaml'
#
# Usage:
#   $ bash dn_push_services.bash [<optional flag>]
#
# Arguments:
#   - [<optional flag>]   Any optional flag from 'dn_execute_compose_over_build_matrix.bash'
#

if [[ $( basename "$(pwd)" ) = build_script ]]; then
    cd ../..
elif [[ $( basename "$(pwd)" ) = dockerized-norlab-scripts ]]; then
    cd ..
fi

# ToDo: on task end >> refactor out the `--build-matrix-file-override .env.build_matrix.dev` line

# docker compose push relevant flags:
#      --ignore-push-failures   Push what it can and ignores images with push failures
bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                 --build-matrix-file-override .env.build_matrix.dev \
                 "$@" -- push
