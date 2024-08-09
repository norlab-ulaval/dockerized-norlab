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
test -d "/project_entrypoints" || n2st::print_msg_error_and_exit "Dir /project_entrypoints is unreachable"

if [[ -f /project_entrypoints/dn_entrypoint.global.init.callback.bash ]]; then
  source /project_entrypoints/dn_entrypoint.global.init.callback.bash || exit 1
else
  n2st::print_msg_warning "dn_entrypoint.global.init.callback.bash unavailable"
fi

if [[ -f /project_entrypoints/project-develop/dn_entrypoint.init.callback.bash ]]; then
  source /project_entrypoints/project-develop/dn_entrypoint.init.callback.bash || exit 1
else
  n2st::print_msg_warning "project-develop/dn_entrypoint.init.callback.bash unavailable"
fi

# ....Release......................................................................................
exec "$@"
