#!/bin/bash
set -e  # exit script if any statement returns a non-true return value

# (CRITICAL) ToDo: NMO-557 test: add missing dn_entrypoint.*.bash unit-tests

# ====DN-project internal logic====================================================================

# ....Load library.................................................................................
if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  echo -e "\033[1;33m[DN trace]\033[0m Execute project-deploy/dn_entrypoint.init.bash"
fi

if [[ $- == *i* ]] || [[ -n "$PS1" ]]; then
    if [[ "${DN_ENTRYPOINT_TRACE_EXECUTION}" == true ]]; then
      echo -e "\033[1;33m[DN trace]\033[0m Interactive shell. Sourcing DN lib is handled via .bashrc"
    fi
else
    if [[ "${DN_ENTRYPOINT_TRACE_EXECUTION}" == true ]]; then
      echo -e "\033[1;33m[DN trace]\033[0m Non-interactive shell. Sourcing DN lib"
    fi
    source /dockerized-norlab/dockerized-norlab-images/container-tools/bash_run_config/.bashrc.dn_non_interactive
fi

test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DN error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }



# ====DN-project user defined logic================================================================

# ....Execute DN-project user callback.............................................................
# Sanity check
test -d "/entrypoints" || n2st::print_msg_error_and_exit "Dir /entrypoints is unreachable"

if [[ -f /entrypoints/dn_entrypoint.global.init.callback.bash ]]; then
  source /entrypoints/dn_entrypoint.global.init.callback.bash || exit 1
else
  n2st::print_msg_warning "dn_entrypoint.global.init.callback.bash unavailable"
fi

if [[ -f /entrypoints/project-deploy/dn_entrypoint.init.callback.bash ]]; then
  source /entrypoints/project-deploy/dn_entrypoint.init.callback.bash || exit 1
else
  n2st::print_msg_warning "project-deploy/dn_entrypoint.init.callback.bash unavailable"
fi

# ....Release......................................................................................
exec "$@"
