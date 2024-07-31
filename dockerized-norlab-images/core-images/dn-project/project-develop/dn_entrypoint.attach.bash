#!/bin/bash
set -e  # exit script if any statement returns a non-true return value

# (CRITICAL) ToDo: NMO-557 test: add missing dn_entrypoint.*.bash unit-tests

# ====DN-project internal logic====================================================================

# ....Load library.................................................................................
source /import_dockerized_norlab_container_tools.bash
n2st::set_which_python3_version && test -n "${PYTHON3_VERSION}" || exit 1
if [[ -z "${PYTHON3_VERSION}" ]]; then
  echo -e "[\033[1;31mERROR\033[0m] $0 | Script import_dockerized_norlab_container_tools.bash failled" 1>&2
fi

if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  n2st::print_msg "Execute $0"
fi

# ====DN-project user defined logic================================================================

# ....Execute DN-project user callback.............................................................
# Sanity check
test -d "/project_entrypoints" || { echo "Dir /project_entrypoints is unreachable" && exit 1 ; }

if [[ -f /project_entrypoints/dn_entrypoint.global.attach.callback.bash ]]; then
  source /project_entrypoints/dn_entrypoint.global.attach.callback.bash || exit 1
else
  echo "dn_entrypoint.global.attach.callback.bash unavailable"
fi

if [[ -f /project_entrypoints/project-develop/dn_entrypoint.attach.callback.bash ]]; then
  source /project_entrypoints/project-develop/dn_entrypoint.attach.callback.bash || exit 1
else
  echo "project-develop/dn_entrypoint.attach.callback.bash unavailable"
fi

# ....Release......................................................................................
exec "$@"
