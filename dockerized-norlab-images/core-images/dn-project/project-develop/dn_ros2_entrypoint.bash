#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

### ToDo: assessment >> no need anymore as we added ros sourcing in Dockerfile.ros2-dn-custom (~/.bashrc and /etc/profile.d/)
## Skip sourcing ros env if it as been already sourced (e.g. by /etc/profile.d/01-dn-source-ros.sh)
#if [[ -z $AMENT_PREFIX_PATH ]]; then
#  # Explicitly source ROS as we removed the dustynv ros_entrypoint.sh from .bashrc
##  ROS_ENV_SETUP="/opt/ros/${ROS_DISTRO}/install/setup.bash"
#  ROS_ENV_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
#  echo "sourcing ${ROS_ENV_SETUP}"
#  source "${ROS_ENV_SETUP}"
#fi

#ROS_DEVEL_ENV_SETUP="${DN_DEV_WORKSPACE}/install/setup.bash"
#echo "sourcing   ${ROS_DEVEL_ENV_SETUP}"
#source "${ROS_DEVEL_ENV_SETUP}"

echo "ROS_DISTRO $ROS_DISTRO"
echo "ROS_ROOT   $ROS_ROOT"

echo
echo -e "Starting container \033[1;37m internal ssh server for IDE remote development workflow on port ${DN_SSH_SERVER_PORT}\033[0m with \033[1;37m user ${DN_SSH_SERVER_USER}\033[0m (default pass: lasagne)"

# sshd flag
# -D : sshd will not detach and does not become a daemon. This allows easy monitoring of sshd.
# -e : sshd will send the output to the standard error instead of the system log.
# -f : config_file
#/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_dockerized_norlab_openssh_server
/usr/sbin/sshd -e -f /etc/ssh/sshd_config_dockerized_norlab_openssh_server

#echo -e "Check if ${DN_PYTHON3_VERSION} is working properly by running \033[1;37m\$ dn_python3_check\033[0m in the container terminal.
#If you need to rebuild your norlab-ros ROS project, use \033[1;37m\$ bash /dn_ros2_rebuild_dev_workspace.bash\033[0m from inside the container,
#"

bash /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash

exec "$@"
