#!/bin/bash
#
# Convenient script for building all images specified in 'docker-compose.dn-dependencies.build.yaml'
#
# Usage:
#   $ bash dn_build_over_single_build_matrix.bash [<optional flag>]
#
# Arguments:
#   - [<optional flag>]   Any optional flag from 'dn_execute_compose_over_build_matrix.bash'
#

clear

if [[ $( basename $(pwd) ) = build_script ]]; then
    cd ../..
elif [[ $( basename $(pwd) ) = dockerized-norlab-scripts ]]; then
    cd ..
fi

bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                      .env.build_matrix.dn-dependencies \
                      "$@" -- build

bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                      .env.build_matrix.dn-control \
                      "$@" -- build
