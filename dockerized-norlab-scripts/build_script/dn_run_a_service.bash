#!/bin/bash
#
# Convenient script
#
# Usage:
#   $ bash dn_run_a_service.bash '<.env.build_matrix.*>' <service> [<optional command>]
#
# Arguments:
#   - <.env.build_matrix.*>  Dotenv build matrix file
#   - <service>              The service to run
#   - [<optional command>]   Any optional flag for docker compose run <service> [command]
#
# Global
#   - Read OPTION_FLAG       Use to quickly add docker flag at runtime
#                            e.g.: $ OPTION_FLAG=--build
#

clear

_DOTENV_BUILD_MATRIX="${1:?' Missing the dotenv build matrix file mandatory argument'}"
shift # Remove argument value

if [[ $( basename "$(pwd)" ) = build_script ]]; then
    cd ../..
elif [[ $( basename "$(pwd)" ) = dockerized-norlab-scripts ]]; then
    cd ..
fi

#OPTION_FLAG='--build'
#OPTION_FLAG='--user pycharm-debugger'

# Notes;
#   - be advised that docker compose run command bypass the container_name field of the .yaml file so you can spin the same service multiple time and all container will have a unique name
bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                "${_DOTENV_BUILD_MATRIX}" \
                --fail-fast \
                -- run "${OPTION_FLAG:-""}" --rm \
                --no-deps "$@"

