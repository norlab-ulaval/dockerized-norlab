#!/bin/bash
set -e  # exit script if any statement returns a non-true return value

# (CRITICAL) ToDo: NMO-557 test: add missing dn_entrypoint.*.bash unit-tests

# ====DN-project internal logic====================================================================
# ....Load library.................................................................................
source /import_dockerized_norlab_container_tools.bash
n2st::set_which_python3_version && test -n "${PYTHON3_VERSION}" || exit 1
if [[ -z "${PYTHON3_VERSION}" ]]; then
  n2st::print_msg_error "$0 | Script import_dockerized_norlab_container_tools.bash failled"
fi

if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  n2st::print_msg "Execute $0"
fi

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
