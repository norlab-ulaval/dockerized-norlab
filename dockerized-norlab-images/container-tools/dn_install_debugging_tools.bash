#!/bin/bash
# =================================================================================================
# Install and setup debug related tools
#
#   1. It checks the required environment variables,
#   2. Installs and sets up an OpenSSH server.
#   4. Creates a dedicated user for debugging daemon e.g.: gdb, pycharm
#   3. Installs/update debugging the necessary packages,
#   5. Sets up user permissions
#   6. Handles various other SSH related configurations.
#
# Usage:
#   $ source ./dn_install_debugging_tools.bash
#
# Global Variables:
# - DN_SSH_SERVER_PORT (Read): The port for the SSH server.
# - DN_SSH_SERVER_USER (Read): Defines the SSH server user.
# - DN_SSH_SERVER_USER_PASSWORD (Read): Password for the SSH server user.
# - DN_PROJECT_GID: (Read) The project group ID.
# - DN_PROJECT_USER: (Read) The project user.
#
# Returns:
# - Exits with a status of 0 on success, non-zero on error.
#
# =================================================================================================
set -e
pushd "$(pwd)" >/dev/null || exit 1

# (CRITICAL) ToDo: unit-test (ref task TASK)

# ....Optional settings............................................................................
_SETUP_DEBUGGER_USER=true
_SETUP_DEBUG_PROJECT_TMP_DIR=false

# ....Source project shell-scripts dependencies....................................................
cd /dockerized-norlab/dockerized-norlab-images/container-tools || exit 1
source import_dockerized_norlab_container_tools.bash

function dn::setup_debugging_tools() {
  # ....Check pre-conditions.........................................................................
  {
    test -n "${DN_SSH_SERVER_PORT:?'Env variable needs to be set and non-empty.'}" && \
    test -n "${DN_SSH_SERVER_USER:?'Env variable needs to be set and non-empty.'}" && \
    test -n "${DN_SSH_SERVER_USER_PASSWORD:?'Env variable needs to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_GID:?'Env variable needs to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}" ;
  } || exit 1

  # ===Service: ssh server===========================================================================
  apt-get update \
      && apt-get install --assume-yes --no-install-recommends \
          openssh-server \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/*

  # ....Setup ssh daemon.............................................................................

  # (CRITICAL) ToDo: (ref task NMO-348 validate that DN_SSH_SERVER_PORT var in 'sshd_config_dockerized_norlab_openssh_server' can be changed at runtime via compose ENV)
  # Inspired from:
  # - https://austinmorlan.com/posts/docker_clion_development/
  # - https://www.allaban.me/posts/2020/08/ros2-setup-ide-docker/
  # - https://github.com/microsoft/docker/blob/master/docs/examples/running_ssh_service.md
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

  # ....Set password for users.......................................................................
  # user:newpassword
  echo "${DN_PROJECT_USER}:${DN_SSH_SERVER_USER_PASSWORD}" | chpasswd
  echo "root:${DN_SSH_SERVER_USER_PASSWORD}" | chpasswd

  # ....Create and setup specialized debugger user.................................................
  if [[ ${_SETUP_DEBUGGER_USER} == true ]]; then
    useradd -m "${DN_SSH_SERVER_USER}" \
      && yes "${DN_SSH_SERVER_USER_PASSWORD}" | passwd "${DN_SSH_SERVER_USER}"

    echo "${DN_SSH_SERVER_USER} ALL=(root) NOPASSWD:ALL" >/etc/sudoers.d/"${DN_SSH_SERVER_USER}"
    chmod 0440 "/etc/sudoers.d/${DN_SSH_SERVER_USER}"
    mkdir -p "/home/${DN_SSH_SERVER_USER}"
    chown -R "${DN_SSH_SERVER_USER}":"${DN_PROJECT_GID}" "/home/${DN_SSH_SERVER_USER}"

    # Add the 'video' groups to new user as it's required for GPU access.
    # (not a problem on norlab-og but mandatory on Jetson device)
    # Ref: https://forums.developer.nvidia.com/t/how-to-properly-create-new-users/68660/2
    usermod -a -G video,sudo "${DN_SSH_SERVER_USER}"

    ## (CRITICAL) ToDo: assessment >> assigning project user primary group to pycharm-debugger (ref task NMO-548)
    #usermod --gid "${DN_PROJECT_GID:?err}" "${DN_SSH_SERVER_USER}"
  fi

  # ....Create the pycharm-debugger user tmp project directory.....................................
  if [[ ${_SETUP_DEBUGGER_USER} == true ]] && [[ ${_SETUP_DEBUG_PROJECT_TMP_DIR} == true ]]; then
    mkdir -p "/home/${DN_SSH_SERVER_USER}/tmp/${DN_PROJECT_GIT_NAME:?err}"
    chown -R "${DN_SSH_SERVER_USER}" "/home/${DN_SSH_SERVER_USER}/tmp/${DN_PROJECT_GIT_NAME}/"
  fi

  # ....add the ros distro source steps to debugger user.............................................
  if [[ ${_SETUP_DEBUGGER_USER} == true ]]; then
    usermod --shell /bin/bash "${DN_SSH_SERVER_USER}"
    # Note: Required for ssh in pycharm-debugger, otherwise it use .sh instead of .bash
    #       and result in not sourcing ros from .bashrc

    ## Option 1: source the root .bashrc in the debugger user
    #( \
    #  echo ""; \
    #  echo "# >>> dockerized-norlab dn-project-debugging-tools "; \
    #  echo "# Step to ensure that ROS related sourcing step are performed in the debugging user "; \
    #  echo "source /root/.bashrc"; \
    #  echo "# <<< dockerized-norlab dn-project-debugging-tools "; \
    #  echo ""; \
    #) >> /home/"${DN_SSH_SERVER_USER}"/.bashrc

    ## Option 2: hardlink the root .bashrc to the debugger user .bashrc
    #sudo ln -fv /root/.bashrc "/home/${DN_SSH_SERVER_USER}/.bashrc"
    ##
    ## Note: The .bashrc files get sourced only for interactive shell, so use 'bash -i' when
    ##       dockerfile build stage require to source ~/.bashrc.
    ##       see https://stackoverflow.com/a/74017557 by Chuck Batson
    ##       e.g. in dockerfile
    ##       SHELL ["/bin/bash", "-i", "-c"]

    ## Option 3 (CURRENT): rely on BASH_ENV env varibale for non-interactive shell
    ## Note: this logic is currently implemented via "dn_bashrc_non_interactive.bash"
  fi

  # ====Jetbrains IDE================================================================================

  ## (!) pytest-cov › "The pytest-cov package, due to technical restrictions, breaks PyCharm's debugger."
  ## see https://www.jetbrains.com/help/pycharm/2023.1/run-debug-configuration-py-test.html
  pip3 uninstall --yes pytest-cov

  ## (!) Hack for solving pytest<->pycharm RuntimeError
  ## `INTERNALERROR> RuntimeError: wrap_controller at 'pytest_runtest_makereport' /usr/local/lib/python3.8/dist-packages/dash/testing/plugin.py:106 did not yield`
  ## dash==2.8.1 › Solution from https://github.com/plotly/dash/issues/2460
  ## Version 2.9.0 is responsible for the pytest<->pycharm RuntimeError
  pip3 install --no-cache-dir 'dash[testing]!=2.9.0'

  ## (!) Skip pytest bogus release 8.1.1
  ## Ref
  ##  - https://github.com/sea-bass/pyrobosim/pull/162
  ##  - https://github.com/ros2/launch/issues/765
  RUN pip3 install 'pytest!=8.1.1'

  n2st::print_msg_warning "Be advised, the ssh daemon still need to be started.
  Curently its done by 'dn_entrypoint.global.init.callback.bash'.
  Ref 'dockerized-norlab-images/core-images/dn-project/project-core/project_entrypoints'.
  "

  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  MSG_ERROR_FORMAT="\033[1;31m"
  MSG_END_FORMAT="\033[0m"
  echo -e "${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dn::setup_debugging_tools
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
