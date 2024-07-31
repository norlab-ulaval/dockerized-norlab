#!/bin/bash
set -e  # exit script if any statement returns a non-true return value

# (CRITICAL) ToDo: NMO-557 test: add missing dn_entrypoint.*.bash unit-tests

# ====DN-project internal logic====================================================================


# ====DN-project user defined logic================================================================
# Sanity check
test -d "/project_entrypoints" || { echo "Dir /project_entrypoints is unreachable" && exit 1 ; }

# ....Execute DN-project user callback.............................................................
if [[ -f /project_entrypoints/dn_entrypoint.global.attach.callback.bash ]]; then
  source /project_entrypoints/dn_entrypoint.global.attach.callback.bash || exit 1
else
  echo "dn_entrypoint.global.attach.callback.bash unavailable"
fi

if [[ -f /project_entrypoints/project-deploy/dn_entrypoint.attach.callback.bash ]]; then
  source /project_entrypoints/project-deploy/dn_entrypoint.attach.callback.bash || exit 1
else
  echo "project-deploy/dn_entrypoint.attach.callback.bash unavailable"
fi

# ....Release......................................................................................
exec "$@"
