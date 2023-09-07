#!/bin/bash
#
# Convenient script for pushing all images specified in 'docker-compose.dockerized-norlab.build.yaml'
#
# Usage:
#   $ bash dn_push_all_images.bash [<optional flag>]
#
# Arguments:
#   - [<optional flag>]   Any optional flag from 'dn_execute_compose_over_build_matrix.bash'
#

if [[ $( basename $(pwd) ) = build_script ]]; then
    cd ../..
elif [[ $( basename $(pwd) ) = dockerized-norlab-scripts ]]; then
    cd ..
fi

# docker compose push relevant flags:
#      --ignore-push-failures   Push what it can and ignores images with push failures
#      --include-deps           Also push images of services declared as dependencies
bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash "$@" -- push
