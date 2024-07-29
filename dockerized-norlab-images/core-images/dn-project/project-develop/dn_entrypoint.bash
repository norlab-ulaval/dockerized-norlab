#!/bin/bash
set -e  # exit script if any statement returns a non-true return value

# ....SSH daemon................................................................................... 
# Check if sshd is running
if [[ -z $(pgrep -x "sshd") ]]; then
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
else
  echo -e "ssh daemon is already running."
fi


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

if [[ -f /project_entrypoints/dn_entrypoint.develop.callback.bash ]]; then
  source /project_entrypoints/dn_entrypoint.develop.callback.bash || exit 1
else
  echo "dn_entrypoint.develop.callback.bash unavailable"
fi

# ....Release......................................................................................
exec "$@"
