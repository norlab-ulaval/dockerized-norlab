
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS debugging-tools

# (CRITICAL) ToDo: on task NMO-584 end >> delete next bloc ↓↓
#ARG IS_TEAMCITY_RUN
#ENV IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:-false}

ENV DN_PROJECT_SERVICE=project-develop
ENV DN_PROJECT_SERVICE_DIR=/dockerized-norlab/project/${DN_PROJECT_SERVICE}


# Set python to print stdin/sderr in real-time for dev/debug
ENV PYTHONUNBUFFERED=1

# ....ssh and gdb server config....................................................................
# Note: Set ENV default at build time via ARG in the dockerfile
ARG DN_SSH_SERVER_PORT=2222
ARG DN_GDB_SERVER_PORT=7777

# ssh port, remaped from default 22 to 2222
ENV DN_SSH_SERVER_PORT=${DN_SSH_SERVER_PORT}
EXPOSE ${DN_SSH_SERVER_PORT}

# gdbserver port, remaped from default 77 to 7777
ENV DN_GDB_SERVER_PORT=${DN_GDB_SERVER_PORT}
EXPOSE ${DN_GDB_SERVER_PORT}

# ...debugging user config.........................................................................
ARG DN_SSH_SERVER_USER=non-interactive-ros2
ENV DN_SSH_SERVER_USER=${DN_SSH_SERVER_USER}
ARG DN_SSH_SERVER_USER_PASSWORD=lasagne


# ....install debugging tools......................................................................
WORKDIR /dockerized-norlab/dockerized-norlab-images/container-tools
COPY --from=context-dn-container-tools ./dn_install_debugging_tools.bash .

### (CRITICAL) ToDo: on DEV ⚠️ task NMO-768 end >> delete this bloc ↓↓
##COPY --from=context-dn-container-tools ./.bashrc.dn_non_interactive ./.bashrc.dn_non_interactive
##COPY --from=context-dn-container-tools ./.bashrc.dn ./.bashrc.dn
#COPY --from=context-dn-container-tools ./import_dockerized_norlab_container_tools.bash /import_dockerized_norlab_container_tools.bash
#RUN <<EOF
#  # ⚠️ DEV TMP -> mod .bashrc.dn_non_interactive without rebuilding depenencies images
#  echo "unset BASH_ENV" >> /dockerized-norlab/dockerized-norlab-images/container-tools/bash_run_config/.bashrc.dn_non_interactive
#
#  echo "Introspec .bashrc.dn_non_interactive ..."
#  cat ./.bashrc.dn_non_interactive
#  echo
#  echo "Introspec .bashrc.dn ..."
#  cat ./.bashrc.dn
#  echo
#EOF


RUN <<EOF
    # ....Pre-condition checks.....................................................................
    n2st::print_msg "Pre-condition checks"
    {
        test $(whoami) == "root" &&  \
        test -n ${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_USER_HOME:?'Env variable need to be set and non-empty.'} && \
        test -d ${DN_PROJECT_USER_HOME} && \
        test -n ${DN_PROJECT_PATH:?'Env variable need to be set and non-empty.'} && \
        test -d ${DN_PROJECT_PATH} && \
        test -n "${DEBIAN_FRONTEND:?'Env variable need to be set and non-empty.'}" && \
        [[ "${DEBIAN_FRONTEND}" == "noninteractive" ]];
    } || n2st::print_msg_error_and_exit "Failed pre-condition checks!"

    # ....Install debugging tools..................................................................
    source ./dn_install_debugging_tools.bash || exit 1
    rm -f ./dn_install_debugging_tools.bash

    # ....Sanity check.............................................................................
    test -f /etc/ssh/sshd_config_dockerized_norlab_openssh_server || exit 1
EOF


# ====Project specific ros setup===================================================================
FROM debugging-tools AS dockerized-norlab-project-internal

# ....DN and DN-project entrypoints and utilities................................................

# ToDo: refactor the rest of the stage as a standalone script that can be executed
# by project-deploy (ref task NMO-558)

# ToDo: Move project-develop/dn_ros2_rebuild_dev_workspace.bash and
# project-develop/dn_fetch_ros_env_variables.bash to a dn-project wide dir so that its available on
# all dn-project service (ref task NMO-558)

# Note: Its easier and more robust to copy DN files to root and then symlink them in project user.
WORKDIR ${DN_PROJECT_SERVICE_DIR}
COPY dn_entrypoint.init.bash dn_entrypoint.attach.bash dn_ros2_rebuild_dev_workspace.bash dn_fetch_ros_env_variables.bash dn_expose_container_env_variables.bash ./

RUN chmod +x dn_entrypoint.init.bash && \
    chmod +x dn_entrypoint.attach.bash && \
    chmod +x dn_ros2_rebuild_dev_workspace.bash && \
    chmod +x dn_fetch_ros_env_variables.bash && \
    chmod +x dn_expose_container_env_variables.bash

# Create required directories
WORKDIR /dn_container_env_variable/

RUN <<EOF
    n2st::print_msg "Create soft link in the home dir"
    ln -s /dn_container_env_variable ${DN_PROJECT_USER_HOME}/dn_container_env_variable
    ln -s ${DN_PROJECT_SERVICE_DIR}/dn_entrypoint.init.bash ${DN_PROJECT_USER_HOME}/dn_entrypoint.init.bash
    ln -s ${DN_PROJECT_SERVICE_DIR}/dn_entrypoint.attach.bash ${DN_PROJECT_USER_HOME}/dn_entrypoint.attach.bash
    ln -s ${DN_PROJECT_SERVICE_DIR}/dn_ros2_rebuild_dev_workspace.bash ${DN_PROJECT_USER_HOME}/dn_ros2_rebuild_dev_workspace.bash
    ln -s ${DN_PROJECT_SERVICE_DIR}/dn_fetch_ros_env_variables.bash ${DN_PROJECT_USER_HOME}/dn_fetch_ros_env_variables.bash
    ln -s ${DN_PROJECT_SERVICE_DIR}/dn_expose_container_env_variables.bash ${DN_PROJECT_USER_HOME}/dn_expose_container_env_variables.bash


    n2st::print_msg "Project directories ownership and simlink"
    {
      chown -R $(id -u ${DN_PROJECT_USER:?err}):$(id -g ${DN_PROJECT_USER}) ${DN_PROJECT_USER_HOME:?err}
      chown -R $(id -u ${DN_PROJECT_USER}):$(id -g ${DN_PROJECT_USER}) ${DN_PROJECT_PATH:?err}
      chown -R $(id -u ${DN_PROJECT_USER}):$(id -g ${DN_PROJECT_USER}) ${DN_DEV_WORKSPACE:?err}
    } || {
      # Collect debugg information on faillure
      pwd
      tree -agu
      tree -agu ${DN_PROJECT_USER_HOME}
      tree -agu ${DN_DEV_WORKSPACE}
      exit 1
    }

    n2st::print_msg "Add project-develop aliases"
    (
      echo ""
      echo "# >>> dockerized-norlab project-develop"
      echo "# Dockerized-NorLab aliases (from project-develop img)"
      echo "alias dn-info='source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash'"
      echo "alias dn-source-ros2='dn::source_ros2'"
      echo "alias dn-source-ros2-underlay-only='dn::source_ros2_underlay_only'"
      echo "alias dn-source-ros2-overlay-only='dn::source_ros2_overlay_only'"
      echo "alias dn-ros2-rebuild-dev-workspace='source /dockerized-norlab/project/project-develop/dn_ros2_rebuild_dev_workspace.bash'"
      echo "alias dn-fetch-ros-env-variables='source /dockerized-norlab/project/project-develop/dn_fetch_ros_env_variables.bash'"
      echo "alias dn-expose-container-env-variables='source /dockerized-norlab/project/project-develop/dn_expose_container_env_variables.bash'"
      echo "# <<< dockerized-norlab project-develop"
      echo ""
    ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash

    # ....Copy environment variable to non-interactive sub shell...................................
    n2st::print_msg "Setup environment variables for run-time non-interactive sub shell..."

#    dn::source_ros2 || exit 1
#
#    # Generate dotenv file
#    {
#      source /dockerized-norlab/project/project-develop/dn_expose_container_env_variables.bash && \
#      test -f "/dn_container_env_variable/.env.dn_expose_${DN_CONTAINER_NAME:?err}";
#    } || n2st::print_msg_error_and_exit "Failed to expose container environment variables!"
#
#    # Override target dotenv
#    cat /dn_container_env_variable/.env.dn_expose_${DN_CONTAINER_NAME:?err} > /dockerized-norlab/dockerized-norlab-images/container-tools/bash_run_config/.env.dn_non_interactive_ros2

    # Override target dotenv
    (
      echo
      echo "ROS_DISTRO=${ROS_DISTRO}"
      echo "ROS_ROOT=${ROS_ROOT}"
      echo "DN_DEV_WORKSPACE=${DN_DEV_WORKSPACE}"
      echo "DN_PROJECT_PATH=${DN_PROJECT_PATH}"
#      echo "PYTHONPATH=${PYTHONPATH}"
#      echo "ROS_PKG=${ROS_PKG}"
#      echo "PATH=${PATH}"
      echo
    ) > /dockerized-norlab/dockerized-norlab-images/container-tools/bash_run_config/.env.dn_non_interactive_ros2

    n2st::print_msg_done "Environment variables for run-time non-interactive sub shell are set."

    # ....Sanity check.............................................................................
    n2st::print_msg "Users configuration sanity check..."
    echo
    echo "id ${DN_PROJECT_USER}: $(id "${DN_PROJECT_USER}")"
    echo
    echo "id ${DN_SSH_SERVER_USER}: $(id "${DN_SSH_SERVER_USER}")"
    echo

    # (CRITICAL) ToDo: validate (ref task TASK)
    getent passwd redleader

    echo "Check non-interactive-ros2..."
    pd_shell_path=$(getent passwd non-interactive-ros2 | awk -F: '{print $7}')
    test "$pd_shell_path" == "/usr/local/bin/bash-dn-non-interactive-ros2"

    getent passwd root

EOF

## (CRITICAL) ToDo: on task NMO-774 end >> delete this bloc ↓↓
#COPY --from=context-dn-container-tools ./bash_run_config/.bashrc.dn_non_interactive_ros2 /dockerized-norlab/dockerized-norlab-images/container-tools/bash_run_config/.bashrc.dn_non_interactive_ros2
#COPY --from=context-dn-container-tools ./bash_wrapper/bash-dn-non-interactive-ros2 /usr/local/bin/bash-dn-non-interactive-ros2
#RUN <<EOF
#    # ////Note: Relevant code copied from DN for DEBUG purposes ⬇︎/////////////////////////////////
#    echo "Configure bash wrappers..."
##    chmod +x /usr/local/bin/bash-dn-buildtime
#    chmod +x /usr/local/bin/bash-dn-non-interactive-ros2
#    echo "Activate DN build time non-interactive shell: /bin/bash -> /usr/local/bin/bash-dn-buildtime ..."
#
#    chown -R "${DN_PROJECT_UID:?err}":"${DN_PROJECT_GID:?err}" "${DN_PATH:?err}"
#EOF


FROM dockerized-norlab-project-internal AS test-ssh-user
USER "${DN_SSH_SERVER_USER}"
SHELL ["/usr/local/bin/bash-dn-non-interactive-ros2", "-c"]
WORKDIR /

RUN <<EOF
    test -n "$( declare -f dn::source_ros2 )" || { echo -e "\033[1;31m[DN error]\033[0m The DN lib is not loaded!" 1>&2 && exit 1; }

    # Note: Seek for the following files:
    #  - "/usr/local/bin/bash-dn-buildtime"
    #  - "/usr/local/bin/bash-dn-non-interactive-ros2"
    echo
    tree -aguL 1 -P bash-dn* /usr/local/bin
    echo


    echo "whoami: $(whoami)"
    echo "id ${DN_PROJECT_USER}: $(id "${DN_PROJECT_USER}")"
    echo "id ${DN_SSH_SERVER_USER}: $(id "${DN_SSH_SERVER_USER}")"
    echo "DN_SSH_SERVER_USER: ${DN_SSH_SERVER_USER}"
    echo "DN_CONTAINER_TOOLS_LOADED: ${DN_CONTAINER_TOOLS_LOADED}"
    echo "DEBIAN_FRONTEND: ${DEBIAN_FRONTEND}"
    tree -aguL 1 /home/

    {
        test $(whoami) == "${DN_SSH_SERVER_USER}" && \
        test "${DN_CONTAINER_TOOLS_LOADED}" == true && \
        test -d /home/${DN_SSH_SERVER_USER} && \
        test -n "${DEBIAN_FRONTEND:?'Env variable need to be set and non-empty.'}" && \
        [[ "${DEBIAN_FRONTEND}" == "noninteractive" ]];
    } || exit 1

    n2st::print_msg "Check that ROS source is available via the ssh daemon user"
    # Note: ROS2 should have been sourced by /usr/local/bin/bash-dn-non-interactive-ros2
    printenv
    {
      test -n "${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" && \
      python -c "import rclpy" ;
    } || exit 1

    echo
    n2st::print_msg "ros2 install pkg sanity check"
    {
        ros2 pkg list && \
        test -n "$(ros2 pkg list | grep -e ros_core)" && \
        test -n "$(ros2 pkg list | grep -e rclpy)";
    } ||  exit 1
EOF

FROM dockerized-norlab-project-internal AS test-project-user
USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/usr/local/bin/bash-dn-buildtime", "-c"]
ARG TARGETPLATFORM
ARG BUILDPLATFORM

WORKDIR /

RUN <<EOF
    {
        test $(whoami) == "${DN_PROJECT_USER}" && \
        test "${DN_CONTAINER_TOOLS_LOADED}" == true && \
        test -d /home/${DN_PROJECT_USER} && \
        test -n "${DEBIAN_FRONTEND:?'Env variable need to be set and non-empty.'}" && \
        [[ "${DEBIAN_FRONTEND}" == "noninteractive" ]];
    } || exit 1

    n2st::print_msg "Check that ROS source is available via the project user"
    # Note: ROS2 should not been sourced implicitly by project user entrypoint or bashrc. It must be explicitly sourced by the project user.
    test -n "$( declare -f dn::source_ros2 )" || { echo -e "\033[1;31m[DN error]\033[0m The DN lib is not loaded!" 1>&2 && exit 1; }
    dn::source_ros2 || exit 1
    printenv
    {
      test -n "${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" && \
      python -c "import rclpy" ;
    } || exit 1

    echo
    n2st::print_msg "ros2 install pkg sanity check"
    echo
    {
        ros2 pkg list && \
        test -n "$(ros2 pkg list | grep -e ros_core)" && \
        test -n "$(ros2 pkg list | grep -e rclpy)";
    } ||  exit 1
EOF

WORKDIR ${DN_PROJECT_USER_HOME}

RUN <<EOF
    {
        test -x "${DN_PROJECT_USER_HOME}/dn_expose_container_env_variables.bash" && \
        source dn_expose_container_env_variables.bash && \
        TMP_DOTENV_EXPOSE_PATH="/dn_container_env_variable/.env.dn_expose_${DN_CONTAINER_NAME}" && \
        test -f "${TMP_DOTENV_EXPOSE_PATH}" && \
        TMP_DOTENV_EXPOSE_USER_PATH="${DN_PROJECT_USER_HOME}/dn_container_env_variable/.env.dn_expose_${DN_CONTAINER_NAME}" && \
        test -f "${TMP_DOTENV_EXPOSE_USER_PATH}" ;
    } || exit 1

    test -x "${DN_PROJECT_USER_HOME}/dn_ros2_rebuild_dev_workspace.bash" || exit 1

    if [[ ${TARGETPLATFORM:?err} != ${BUILDPLATFORM:?err} ]]; then
      source ./dn_ros2_rebuild_dev_workspace.bash 'virtualization' || exit 1
    else
      source ./dn_ros2_rebuild_dev_workspace.bash 'native' || exit 1
    fi

    {
        test -x "${DN_PROJECT_USER_HOME}/dn_entrypoint.init.bash" && \
        test -x "${DN_PROJECT_USER_HOME}/dn_entrypoint.attach.bash" && \
        test -d "${DN_PROJECT_USER_HOME}" && \
        test -d "${DN_PROJECT_PATH}" && \
        test -d "${DN_DEV_WORKSPACE}"&& \
        test -d "${DN_PROJECT_USER_HOME}/$(basename ${DN_DEV_WORKSPACE})" && \
        test -d "${DN_PROJECT_USER_HOME}/${DN_PROJECT_GIT_NAME}" ;
    } || exit 1

    {
        test -x "${DN_PROJECT_USER_HOME}/dn_fetch_ros_env_variables.bash" && \
        source ./dn_fetch_ros_env_variables.bash ;
    } || exit 1

    echo "container-tools sanity check"
    {
        test -f /dockerized-norlab/dockerized-norlab-images/container-tools/bash_run_config/.bashrc.dn && \
        test -f /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash && \
        test -f /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash && \
        source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash ;
    } || exit 1
EOF

# unset entrypoint for running test stage
ENTRYPOINT [ "" ]

FROM dockerized-norlab-project-internal AS final
USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/usr/local/bin/bash-dn-buildtime", "-c"]
RUN <<EOF
    echo "Rebuilding rosdep db for project user ${DN_PROJECT_USER}..."
    rosdep update --rosdistro ${ROS_DISTRO:?err} || exit 1
EOF

# Switch to default shell
SHELL ["/bin/bash", "-c"]

# Unset DEBIAN_FRONTEND (which is set globaly in base DN images) before run stage.
ENV DEBIAN_FRONTEND=""

#ENV BASH_ENV="/dockerized-norlab/dockerized-norlab-images/container-tools/bash_run_config/.bashrc.dn_non_interactive_ros2"

#WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
WORKDIR ${DN_PROJECT_USER_HOME:?'environment variable is not set'}
ENTRYPOINT [ "/dockerized-norlab/project/project-develop/dn_entrypoint.init.bash" ]
CMD [ "bash" ]




