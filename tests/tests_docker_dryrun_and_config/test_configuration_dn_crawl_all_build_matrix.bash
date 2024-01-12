#!/bin/bash

# ....path resolution logic........................................................................
_PATH_TO_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
NABO_ROOT_DIR="$(dirname "${_PATH_TO_SCRIPT}")"
cd "${NABO_ROOT_DIR}/../../"

# ====begin========================================================================================

export NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG="config --quiet"

#FLAGS=( '--fail-fast' '--' 'build' '--dry-run' )
#bash dockerized-norlab-scripts/build_script/dn_build_all.bash ${FLAGS[@]}

#bash dockerized-norlab-scripts/build_script/dn_build_all.bash --fail-fast -- build --dry-run dependencies dependencies-doc

bash dockerized-norlab-scripts/build_script/dn_build_all.bash --fail-fast
