#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

echo
echo -e "Starting container \033[1;37mssh server on port ${DS_PYCHARM_DEV_SERVER_PORT}\033[0m with \033[1;37muser ${DS_PYCHARM_DEV_USER}\033[0m (default pass: lasagne)"
# sshd flag
# -D : sshd will not detach and does not become a daemon. This allows easy monitoring of sshd.
# -e : sshd will send the output to the standard error instead of the system log.
# -f : config_file
#/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_dockerized_snow_openssh_server
/usr/sbin/sshd -e -f /etc/ssh/sshd_config_dockerized_snow_openssh_server

echo -e "To connect remotely to the container:
    $ ssh -p ${DS_PYCHARM_DEV_SERVER_PORT} ${DS_PYCHARM_DEV_USER}@$(hostname -I | awk '{print $1}')
    $ sftp -P ${DS_PYCHARM_DEV_SERVER_PORT} openssh-$(hostname -I | awk '{print $1}')
    $ scp -P ${DS_PYCHARM_DEV_SERVER_PORT} /path/to/foo ${DS_PYCHARM_DEV_USER}@$(hostname -I | awk '{print $1}'):/dest/
"

#echo -e "Check if ${DS_PYTHON3_VERSION} is working properly by running \033[1;37m\$ python3 /ros2_ws/src/${DS_TARGET_PROJECT_SRC_REPO}/src/container_related/try_pytorch.py\033[0m in the container terminal.
#If you need to rebuild your norlab-ros2 ROS project, use \033[1;37m\$ bash /rebuild_norlab_mppi.bash\033[0m from inside the container,
#"

echo -e "Container information (ds_info command):" && bash /ds_info.bash

echo -e "In container available alias:

  ds_info
  ds_python3_check
  ds_rebuild_norlab_mppi
  ds_fetch_ros_env
"

exec "$@"
