#!/bin/bash
#
# Convenient script for stopping services specified in 'docker-compose.dn-dependencies.build.yaml'.
# All will be stopped if no argument are given.
#
# Usage:
#   $ bash dn_down.bash '<.env.build_matrix.*>' [<service>]
#
# Arguments:
#   - <.env.build_matrix.*>  Dotenv build matrix file
#   - [services]              The service to stop
#

_DOTENV_BUILD_MATRIX="${1:?' Missing the dotenv build matrix file mandatory argument'}"
shift # Remove argument value


if [[ $( basename "$(pwd)" ) = build_script ]]; then
    cd ../..
elif [[ $( basename "$(pwd)" ) = dockerized-norlab-scripts ]]; then
    cd ..
fi


bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash "${_DOTENV_BUILD_MATRIX}" \
                                                            --fail-fast \
                                                            -- down "$@"

