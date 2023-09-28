#!/bin/bash
set -e

# (Priority) ToDo: unit-test (ref task TASK)

# ....Check pre-conditions.........................................................................
echo "${DN_SSH_SERVER_PORT:?'Build argument needs to be set and non-empty.'}"
echo "${DN_SSH_SERVER_USER:?'Build argument needs to be set and non-empty.'}"
echo "${DN_SSH_SERVER_USER_PASSWORD:?'Build argument needs to be set and non-empty.'}"

# ===Service: ssh server===========================================================================
apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        openssh-server \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# (CRITICAL) ToDo: (ref task NMO-348 validate that DN_SSH_SERVER_PORT var in 'sshd_config_dockerized_norlab_openssh_server' can be changed at runtime via compose ENV)
# Inspired from:
# - https://austinmorlan.com/posts/docker_clion_development/
# - https://www.allaban.me/posts/2020/08/ros2-setup-ide-docker/
# - https://github.com/microsoft/docker/blob/master/docs/examples/running_ssh_service.md
# Note: Use $"VAR" instead of "$VAR" to write the $VAR in the bashrc without substitution
# ToDo: test › expect no substitution
#  echo "Port $"DN_SSH_SERVER_PORT""; \
( \
  echo "LogLevel DEBUG2"; \
  echo "PermitRootLogin yes"; \
  echo "PasswordAuthentication yes"; \
  echo "Port ${DN_SSH_SERVER_PORT}"; \
  echo "Subsystem sftp /usr/lib/openssh/sftp-server"; \
) > /etc/ssh/sshd_config_dockerized_norlab_openssh_server \
&& mkdir /run/sshd

#  echo "PermitUserEnvironment yes"; \

# SSH login fix. Otherwise user is kicked off after login
# Ref https://github.com/microsoft/docker/blob/master/docs/examples/running_ssh_service.md
sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

useradd -m "${DN_SSH_SERVER_USER}" \
  && yes "${DN_SSH_SERVER_USER_PASSWORD}" | passwd "${DN_SSH_SERVER_USER}"
# Add the 'video' groups to new user as it's required for GPU access.
# (not a problem on norlab-og but mandatory on Jetson device)
# Ref: https://forums.developer.nvidia.com/t/how-to-properly-create-new-users/68660/2
usermod -a -G video,sudo "${DN_SSH_SERVER_USER}"

# ...root config...................................................................................
# user:newpassword
echo "root:${DN_SSH_SERVER_USER_PASSWORD}" | chpasswd

# (Priority) ToDo: test without usermod
#usermod --shell /bin/bash "${DN_SSH_SERVER_USER}"

# ....add the ros distro source steps to debugger user..............................................

## Option 1: source the root .bashrc in the debugger user
#( \
#  echo ""; \
#  echo "# >>> dockerized-norlab dn-project-debugging-tools "; \
#  echo "# Step to ensure that ROS related sourcing step are performed in the debugging user "; \
#  echo "source /root/.bashrc"; \
#  echo "# <<< dockerized-norlab dn-project-debugging-tools "; \
#  echo ""; \
#) >> /home/"${DN_SSH_SERVER_USER}"/.bashrc

# Option 2: hardlink the root .bashrc to the debugger user .bashrc
#sudo ln -fv /root/.bashrc "/home/${DN_SSH_SERVER_USER}/.bashrc"

# Option 3 (CURRENT): rely on /etc/profile.d/
# Note: this logic is currently implemented in dockerized-norlab-images/core-images/dn-project/project-develop/Dockerfile

# ====Jetbrains IDE================================================================================

## (!) pytest-cov › "The pytest-cov package, due to technical restrictions, breaks PyCharm's debugger."
## see https://www.jetbrains.com/help/pycharm/2023.1/run-debug-configuration-py-test.html
pip3 uninstall --yes pytest-cov

## (!) Hack for solving pytest<->pycharm RuntimeError
## `INTERNALERROR> RuntimeError: wrap_controller at 'pytest_runtest_makereport' /usr/local/lib/python3.8/dist-packages/dash/testing/plugin.py:106 did not yield`
## dash==2.8.1 › Solution from https://github.com/plotly/dash/issues/2460
## Version 2.9.0 is responsible for the pytest<->pycharm RuntimeError
pip3 install --no-cache-dir 'dash[testing]!=2.9.0'


# Note: the ssh daemon still need to be started.
#  Curently its done by 'dn_ros2_entrypoint.bash' # (CRITICAL) ToDo: assessment

return 0
