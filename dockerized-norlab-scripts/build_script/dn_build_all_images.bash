#!/bin/bash
#
# Convenient script for building all images specified in 'docker-compose.dockerized-norlab.build.yaml'
#
# Usage:
#   $ bash dn_build_all_images.bash [<optional flag>]
#
# Arguments:
#   - [<optional flag>]   Any optional flag from 'dn_execute_compose_over_build_matrix.bash'
#

if [[ $( basename $(pwd) ) = build_script ]]; then
    cd ../..
elif [[ $( basename $(pwd) ) = dockerized-norlab-scripts ]]; then
    cd ..
fi

bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash "$@" -- build
