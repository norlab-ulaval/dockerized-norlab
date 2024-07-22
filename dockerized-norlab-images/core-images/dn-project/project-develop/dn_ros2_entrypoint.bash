#!/bin/bash
set -e  # exit script if any statement returns a non-true return value

echo
echo -e "Starting container \033[1;37m internal ssh server for IDE remote development workflow on port ${DN_SSH_SERVER_PORT}\033[0m with \033[1;37m user ${DN_SSH_SERVER_USER}\033[0m (default pass: lasagne)"

#LAUNCH_SSN_DAEMON=( '/usr/sbin/sshd' '-D' '-e' '-f' '/etc/ssh/sshd_config_dockerized_norlab_openssh_server' )
LAUNCH_SSN_DAEMON=( '/usr/sbin/sshd' '-e' '-f' '/etc/ssh/sshd_config_dockerized_norlab_openssh_server' )
# Note on sshd flags:
# -D : sshd will not detach and does not become a daemon. This allows easy monitoring of sshd.
# -e : sshd will send the output to the standard error instead of the system log.
# -f : config_file
if [[ $(whoami) == "root" ]]; then
  "${LAUNCH_SSN_DAEMON[@]}"
else
  sudo "${LAUNCH_SSN_DAEMON[@]}"
fi

#echo -e "Check if ${DN_PYTHON3_VERSION} is working properly by running \033[1;37m\$ dn_python3_check\033[0m in the container terminal.
#If you need to rebuild your norlab-ros ROS project, use \033[1;37m\$ bash /dn_ros2_rebuild_dev_workspace.bash\033[0m from inside the container,
#"

source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash

cd "${DN_PROJECT_PATH:?"Env variable need to be set and non-empty"}" || exit 1

if [[ -f /dn_entrypoint.project.callback.bash ]]; then
  source /dn_entrypoint.project.callback.bash
fi

exec "$@"
