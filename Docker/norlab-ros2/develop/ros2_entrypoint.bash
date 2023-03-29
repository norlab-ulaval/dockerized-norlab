#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

echo
echo -e "Starting container \033[1;37m internal ssh server for IDE remote development workflow on port ${DS_PYCHARM_DEV_SERVER_PORT}\033[0m with \033[1;37m user ${DS_PYCHARM_DEV_USER}\033[0m (default pass: lasagne)"

# sshd flag
# -D : sshd will not detach and does not become a daemon. This allows easy monitoring of sshd.
# -e : sshd will send the output to the standard error instead of the system log.
# -f : config_file
#/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_dockerized_snow_openssh_server
/usr/sbin/sshd -e -f /etc/ssh/sshd_config_dockerized_snow_openssh_server

#echo -e "Check if ${DS_PYTHON3_VERSION} is working properly by running \033[1;37m\$ python3 /ros2_ws/src/${DN_TARGET_PROJECT_SRC_REPO}/src/container_related/try_pytorch.py\033[0m in the container terminal.
#If you need to rebuild your norlab-ros2 ROS project, use \033[1;37m\$ bash /ros2_rebuild_dev_workspace.bash\033[0m from inside the container,
#"

bash /dn_info.bash

exec "$@"
