#!/bin/bash
set -e  # exit script if any statement returns a non-true return value

# (CRITICAL) ToDo: NMO-557 test: add missing dn_entrypoint.*.bash unit-tests

# ====DN-project internal logic====================================================================
# ....Load library.................................................................................
## (CRITICAL) ToDo: validate >> deleting DN lib import ↓ (ref task NMO-770)
#source /import_dockerized_norlab_container_tools.bash

if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  echo -e "\033[1;33m[DN trace]\033[0m Execute dn_entrypoint.init.bash"
fi

if [[ $- == *i* ]]; then
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


# ....SSH daemon...................................................................................
# Check if sshd is running
n2st::print_msg "Starting container internal ssh server for IDE remote development workflow on port ${MSG_DIMMED_FORMAT}${DN_SSH_SERVER_PORT}${MSG_END_FORMAT} with user ${MSG_DIMMED_FORMAT}${DN_SSH_SERVER_USER}${MSG_END_FORMAT}"

#LAUNCH_SSN_DAEMON=( '/usr/sbin/sshd' '-D' '-e' '-f' '/etc/ssh/sshd_config_dockerized_norlab_openssh_server' )
LAUNCH_SSN_DAEMON=( '/usr/sbin/sshd' '-e' '-f' '/etc/ssh/sshd_config_dockerized_norlab_openssh_server' )
# Note on sshd flags:
# -D : sshd will not detach and does not become a daemon. This allows easy monitoring of sshd.
# -e : sshd will send the output to the standard error instead of the system log.
# -f : config_file
if [[ $(whoami) == "root" ]]; then
  "${LAUNCH_SSN_DAEMON[@]}"
else
  n2st::print_msg "Launching the ssh daemon in a subshell with sudo priviledge instead of the $(whoami) shell"
  sudo bash -c "${LAUNCH_SSN_DAEMON[*]}"
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
