
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS debugging-tools

# (CRITICAL) ToDo: on task NMO-584 end >> delete next bloc ↓↓
#ARG IS_TEAMCITY_RUN
#ENV IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:-false}

ENV DN_PROJECT_SERVICE=project-develop
ENV DN_PROJECT_SERVICE_DIR=/dockerized-norlab/project/${DN_PROJECT_SERVICE}

RUN <<EOF
    n2st::print_msg "Pre-condition checks"
    {
        test $(whoami) == "root" &&  \
        test -n ${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_USER_HOME:?'Env variable need to be set and non-empty.'} && \
        test -d ${DN_PROJECT_USER_HOME} && \
        test -n ${DN_PROJECT_PATH:?'Env variable need to be set and non-empty.'} && \
        test -d ${DN_PROJECT_PATH} ;
    } || exit 1
EOF

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
ARG DN_SSH_SERVER_USER=pycharm-debugger
ENV DN_SSH_SERVER_USER=${DN_SSH_SERVER_USER}
ARG DN_SSH_SERVER_USER_PASSWORD=lasagne

# ....install debugging tools......................................................................
WORKDIR /dockerized-norlab/dockerized-norlab-images/container-tools
COPY --from=context-dn-container-tools ./dn_install_debugging_tools.bash .
RUN source ./dn_install_debugging_tools.bash \
    && rm -f ./dn_install_debugging_tools.bash \
    && test -f /etc/ssh/sshd_config_dockerized_norlab_openssh_server || exit 1


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
      echo "alias dn_info='source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash'"
      echo "alias dn_ros2_rebuild_dev_workspace='source /dockerized-norlab/project/project-develop/dn_ros2_rebuild_dev_workspace.bash'"
      echo "alias dn_fetch_ros_env_variables='source /dockerized-norlab/project/project-develop/dn_fetch_ros_env_variables.bash'"
      echo "alias dn_expose_container_env_variables='source /dockerized-norlab/project/project-develop/dn_expose_container_env_variables.bash'"
      echo "# <<< dockerized-norlab project-develop"
      echo ""
    ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash
EOF

# ....TMP DEV introspection........................................................................
## (CRITICAL) ToDo: on task end >> mute next bloc ↓↓
#RUN <<EOF
#    # TMP DEV introspection
#    cat /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash
#    echo -e "\n${0}: not implemented error\n" && exit 1
#EOF
# ........................................................................TMP DEV introspection....

# ....TMP DEV dn_info.bash refactor................................................................
### (CRITICAL) ToDo: on task end >> mute next bloc ↓↓
#WORKDIR /dockerized-norlab/dockerized-norlab-images/container-tools
#COPY --from=context-dn-container-tools ./dn_info.bash .
# ................................................................TMP DEV dn_info.bash refactor....


FROM dockerized-norlab-project-internal AS test-ssh-user
USER "${DN_SSH_SERVER_USER}"
SHELL ["/bin/bash", "-c"]
WORKDIR /

RUN <<EOF
    {
        test $(whoami) == "${DN_SSH_SERVER_USER}" && \
        test -d /home/${DN_SSH_SERVER_USER} ;
    } || exit 1

    n2st::print_msg "Check that ROS source is available via the ssh daemon user"
    printenv
    {
      test -n "${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" && \
      python -c "import rclpy" ;
    } || exit 1

    n2st::print_msg "ros2 install pkg sanity check"
    {
        test -n "$(ros2 pkg list | grep -e ros_core)" &&  \
        test -n "$(ros2 pkg list | grep -e rclpy)" &&  \
        test -n "$(ros2 pkg list | grep -e ackermann_msgs)" &&  \
        test -n "$(ros2 pkg list | grep -e vesc_msgs)" ;
    } ||  exit 1
EOF

FROM dockerized-norlab-project-internal AS test-project-user
USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/bin/bash", "-c"]
ARG TARGETPLATFORM
ARG BUILDPLATFORM
WORKDIR /

RUN <<EOF
    {
        test $(whoami) == "${DN_PROJECT_USER}" && \
        test -d /home/${DN_PROJECT_USER} ;
    } || exit 1
EOF

RUN <<EOF
    n2st::print_msg "Check that ROS source is available via the ssh daemon user"
    printenv
    {
      test -n "${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" && \
      python -c "import rclpy" ;
    } || exit 1

    n2st::print_msg "ros2 install pkg sanity check"
    {
        test -n "$(ros2 pkg list | grep -e ros_core)" &&  \
        test -n "$(ros2 pkg list | grep -e rclpy)" &&  \
        test -n "$(ros2 pkg list | grep -e ackermann_msgs)" &&  \
        test -n "$(ros2 pkg list | grep -e vesc_msgs)" ;
    } ||  exit 1
EOF

WORKDIR ${DN_PROJECT_USER_HOME}
RUN <<EOF
    {
        test -x "${DN_PROJECT_USER_HOME}/dn_expose_container_env_variables.bash" && \
        bash dn_expose_container_env_variables.bash && \
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
        test -f /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bashrc.bash && \
        test -f /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash && \
        test -f /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash && \
        source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash ;
    } || exit 1
EOF

# unset entrypoint for running test stage
ENTRYPOINT [ "" ]

FROM dockerized-norlab-project-internal AS final

USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/bin/bash", "-c"]

#WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
WORKDIR ${DN_PROJECT_USER_HOME:?'environment variable is not set'}
ENTRYPOINT [ "/dockerized-norlab/project/project-develop/dn_entrypoint.init.bash" ]
CMD [ "bash" ]




