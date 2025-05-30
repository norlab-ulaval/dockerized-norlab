#!/bin/bash

# ....path resolution logic........................................................................
_PATH_TO_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
DN_ROOT_DIR="$( realpath "$(dirname "${_PATH_TO_SCRIPT}")/../../" )"
cd "${DN_ROOT_DIR}"

# ====begin========================================================================================

export NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG="build --dry-run --no-cache"

bash dockerized-norlab-scripts/build_script/dn_build_all.bash
exit $?
