#!/bin/bash
# =================================================================================================
#
# Convenient script for pushing all images specified in a 'docker-compose.<project-name>.build.yaml'
#
# Usage:
#   $ bash dn_push_services.bash '<.env.build_matrix.*>' [<optional flag>]
#
# Arguments:
#   - <.env.build_matrix.*>  Dotenv build matrix file
#   - [<optional flag>]   Any optional flag from 'dn_execute_compose_over_build_matrix.bash'
#
# =================================================================================================

_DOTENV_BUILD_MATRIX="${1:?' Missing the dotenv build matrix file mandatory argument'}"
shift # Remove argument value


if [[ $( basename "$(pwd)" ) = build_script ]]; then
    cd ../..
elif [[ $( basename "$(pwd)" ) = dockerized-norlab-scripts ]]; then
    cd ..
fi


# (NICE TO HAVE) ToDo: refactor to use 'dn_build_all.bash' with NBS_OVERRIDE_ADD_DOCKER_FLAG, NBS_OVERRIDE_DOCKER_CMD and NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY

bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash "${_DOTENV_BUILD_MATRIX}"\
                 "$@" -- push

# docker compose push relevant flags:
#      --ignore-push-failures   Push what it can and ignores images with push failures
