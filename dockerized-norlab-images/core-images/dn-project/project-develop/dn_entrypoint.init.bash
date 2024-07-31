#!/bin/bash
set -e  # exit script if any statement returns a non-true return value

# (CRITICAL) ToDo: NMO-557 test: add missing dn_entrypoint.*.bash unit-tests

# ====DN-project internal logic====================================================================

# ....SSH daemon...................................................................................
# Check if sshd is running
echo -e "\nStarting container \033[1;37m internal ssh server for IDE remote development workflow on port ${DN_SSH_SERVER_PORT}\033[0m with \033[1;37m user ${DN_SSH_SERVER_USER}\033[0m (default pass: lasagne)"

#LAUNCH_SSN_DAEMON=( '/usr/sbin/sshd' '-D' '-e' '-f' '/etc/ssh/sshd_config_dockerized_norlab_openssh_server' )
LAUNCH_SSN_DAEMON=( '/usr/sbin/sshd' '-e' '-f' '/etc/ssh/sshd_config_dockerized_norlab_openssh_server' )
# Note on sshd flags:
# -D : sshd will not detach and does not become a daemon. This allows easy monitoring of sshd.
# -e : sshd will send the output to the standard error instead of the system log.
# -f : config_file
if [[ $(whoami) == "root" ]]; then
  "${LAUNCH_SSN_DAEMON[@]}"
else
  echo "Launch the ssh daemon in a subshell as root"
  sudo bash -c "${LAUNCH_SSN_DAEMON[*]}"
fi

# ====DN-project user defined logic================================================================

# ....Load library.................................................................................
source /import_dockerized_norlab_container_tools.bash
n2st::set_which_python3_version && test -n "${PYTHON3_VERSION}" || exit 1
if [[ -n "${PYTHON3_VERSION}" ]]; then
  echo -e "[\033[1;31mERROR\033[0m] dn_entrypoint.init.bash | Script import_dockerized_norlab_container_tools.bash failled" 1>&2
fi

# ....Execute DN-project user callback.............................................................
# Sanity check
test -d "/project_entrypoints" || { echo "Dir /project_entrypoints is unreachable" && exit 1 ; }

if [[ -f /project_entrypoints/dn_entrypoint.global.init.callback.bash ]]; then
  source /project_entrypoints/dn_entrypoint.global.init.callback.bash || exit 1
else
  echo "dn_entrypoint.global.init.callback.bash unavailable"
fi

if [[ -f /project_entrypoints/project-develop/dn_entrypoint.init.callback.bash ]]; then
  source /project_entrypoints/project-develop/dn_entrypoint.init.callback.bash || exit 1
else
  echo "project-develop/dn_entrypoint.init.callback.bash unavailable"
fi

# ....Release......................................................................................
exec "$@"
