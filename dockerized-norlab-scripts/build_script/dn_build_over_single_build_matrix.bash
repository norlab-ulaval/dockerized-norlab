#!/bin/bash
# =================================================================================================
#
# Convenient script for building all images specified in in a dotenv build matrix file
#
# Usage:
#   $ bash dn_build_over_single_build_matrix.bash '<.env.build_matrix.*>' [<optional-flag>]
#
# Arguments:
#   - <.env.build_matrix.*>  Dotenv build matrix file
#   - [<optional-flag>]   Any optional flag from 'dn_execute_compose_over_build_matrix.bash'
#
# Global
#   - Read NBS_OVERRIDE_ADD_DOCKER_FLAG    Use to quickly add docker flag at runtime
#                               e.g.: $ NBS_OVERRIDE_ADD_DOCKER_FLAG=--push myService && bash dn_build_over_single_build_matrix.bash
#
# =================================================================================================
clear

# ====Begin========================================================================================
DOCKER_CMD=build
# ....manual config.................................................................................

#DOCKER_CMD="push --ignore-push-failures"
#DOCKER_CMD="up --build"
#DOCKER_CMD="build --dry-run"

# ....setup........................................................................................
_DOTENV_BUILD_MATRIX="${1:?' Missing the dotenv build matrix file mandatory argument'}"
shift # Remove argument value

# Note: 'NBS_OVERRIDE_ADD_DOCKER_FLAG' is set via commandline for convenience
_DOCKER_COMMAND_W_FLAGS="$DOCKER_CMD ${NBS_OVERRIDE_ADD_DOCKER_FLAG:-""}"

#_DOCKER_COMMAND_W_FLAGS="build --dry-run" # (CRITICAL) ToDo: on task end >> delete this line ←

# ....execute......................................................................................
bash ./dockerized-norlab-scripts/build_script/dn_execute_compose_over_build_matrix.bash \
                                    "$_DOTENV_BUILD_MATRIX" \
                                    "$@" -- "$_DOCKER_COMMAND_W_FLAGS"
