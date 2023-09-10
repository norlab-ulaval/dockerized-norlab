#!/bin/bash
#
# Convenient script for building all images specified in 'docker-compose.dn-dependencies.build.yaml'
#
# Usage:
#   $ bash dn_build_all_images.bash '<.env.build_matrix.*>' [<optional flag>]
#
# Arguments:
#   - <.env.build_matrix.*>  Dotenv build matrix file
#   - [<optional flag>]   Any optional flag from 'dn_execute_compose_over_build_matrix.bash'
#

clear

DOTENV_BUILD_MATRIX="${1:?' Missing the dotenv build matrix file mandatory argument'}"
shift # Remove argument value

if [[ $( basename $(pwd) ) = build_script ]]; then
    cd ../..
elif [[ $( basename $(pwd) ) = dockerized-norlab-scripts ]]; then
    cd ..
fi

bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash "${DOTENV_BUILD_MATRIX}" "$@" -- build
