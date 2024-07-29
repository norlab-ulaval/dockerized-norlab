#!/bin/bash
set -e  # exit script if any statement returns a non-true return value

# ....User feedback................................................................................
source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash

# ....Execute DN-project user callback.............................................................

# Sanity check
test -d "/project_entrypoints" || { echo "Dir /project_entrypoints is unreachable" && exit 1 ; }

if [[ -f /project_entrypoints/dn_entrypoint.main.callback.bash ]]; then
  source /project_entrypoints/dn_entrypoint.main.callback.bash || exit 1
else
  echo "dn_entrypoint.main.callback.bash unavailable"
fi

if [[ -f /project_entrypoints/dn_entrypoint.deploy.callback.bash ]]; then
  source /project_entrypoints/dn_entrypoint.deploy.callback.bash || exit 1
else
  echo "dn_entrypoint.deploy.callback.bash unavailable"
fi

# ....Release......................................................................................
exec "$@"
