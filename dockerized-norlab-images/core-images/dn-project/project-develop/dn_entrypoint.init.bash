#!/bin/bash
set -e  # exit script if any statement returns a non-true return value

# (CRITICAL) ToDo: NMO-557 test: add missing dn_entrypoint.*.bash unit-tests

# ====DN-project internal logic====================================================================

# ....Load library.................................................................................
if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  echo -e "\033[1;33m[DN trace]\033[0m Execute project-develop/dn_entrypoint.init.bash"
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

if [[ -f /entrypoints/project-develop/dn_entrypoint.init.callback.bash ]]; then
  source /entrypoints/project-develop/dn_entrypoint.init.callback.bash || exit 1
else
  n2st::print_msg_warning "project-develop/dn_entrypoint.init.callback.bash unavailable"
fi

# ====SSH daemon===================================================================================
# Check if sshd is running
n2st::print_msg "Starting container internal ssh server on port ${MSG_DIMMED_FORMAT}${DN_SSH_SERVER_PORT}${MSG_END_FORMAT} for IDE remote development workflow"

# Usage:
#   $ /usr/sbin/sshd [-D] -e -f /etc/ssh/sshd_config_dockerized_norlab_openssh_server
#
# Note on sshd flags:
# -D : sshd will not detach and does not become a daemon. This allows easy monitoring of sshd.
# -e : sshd will send the output to the standard error instead of the system log.
# -f config_file: use configuration file
LAUNCH_SSH_DAEMON=( /usr/sbin/sshd )
if [[ ${DN_SSH_DAEMON_NO_DETACH} == true ]]; then
  LAUNCH_SSH_DAEMON+=( -D )
  n2st::print_msg_warning "Be advised, launching the ssh daemon in NO-DETACH mode. Execute open another terminal window and execute ${MSG_DIMMED_FORMAT}dna up${MSG_END_FORMAT} to connect to the container."
  read -r -n 1 -p "Press any key to continue"
fi
LAUNCH_SSH_DAEMON+=( -e )
LAUNCH_SSH_DAEMON+=( -f /etc/ssh/sshd_config_dockerized_norlab_openssh_server )
n2st::print_msg "Launching ssh daemon with the following flags ${MSG_DIMMED_FORMAT}${LAUNCH_SSH_DAEMON[*]}${MSG_END_FORMAT}"
if [[ $(whoami) == "root" ]]; then
  "${LAUNCH_SSH_DAEMON[@]}" || n2st::print_msg_warning "Something went wrong with the ssh daemon!"
else
  #n2st::print_msg "Be advised, launching the ssh daemon in a subshell with sudo priviledge instead of the $(whoami) current shell"
  sudo bash -c "${LAUNCH_SSH_DAEMON[*]} || { echo \"\033[1;31m[DN error]\033[0m Something went wrong with the ssh daemon!\" 1>&2; exit 1; }"
fi

# ....Release......................................................................................
exec "$@"
