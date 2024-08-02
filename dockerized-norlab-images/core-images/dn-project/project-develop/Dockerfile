
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS debugging-tools

ARG IS_TEAMCITY_RUN
ENV IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:-false}

RUN echo "Pre-condition checks" && { \
      test $(whoami) == "root" &&  \
      test -n ${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'} && \
      test -d /home/${DN_PROJECT_USER} && \
      test -d ${DN_PROJECT_PATH} ; \
    } || exit 1

# Set python to print stdin/sderr in real-time for dev/debug
ENV PYTHONUNBUFFERED=1

# ....ssh and gdb server config....................................................................
# ssh port, remaped from default 22 to 2222
ENV DN_SSH_SERVER_PORT=${DN_SSH_SERVER_PORT:-2222}
EXPOSE ${DN_SSH_SERVER_PORT}

# gdbserver port, remaped from default 77 to 7777
ENV DN_GDB_SERVER_PORT=${DN_GDB_SERVER_PORT:-7777}
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

## Create the remote interpreter directory
#WORKDIR /home/${DN_SSH_SERVER_USER}/tmp/${DN_PROJECT_GIT_NAME:?err}
#RUN chown -R ${DN_SSH_SERVER_USER} /home/${DN_SSH_SERVER_USER}/tmp/${DN_PROJECT_GIT_NAME}/


# ====Project specific ros setup===================================================================
FROM debugging-tools AS project-specific-ros-setup

# ....DN and DN-project entrypoints and utilities................................................

# ToDo: refactor the rest of the stage as a standalone script that can be executed
# by project-deploy (ref task NMO-558)

# ToDo: Move project-develop/dn_ros2_rebuild_dev_workspace.bash and
# project-develop/dn_fetch_ros_env_variables.bash to a dn-project wide dir so that its available on
# all dn-project service (ref task NMO-558)


# Note: Its easier and more robust to copy DN files to root and then symlink them in project user.
ENV DN_PROJECT_SERVICE_DIR=/dockerized-norlab/project/project-develop
WORKDIR ${DN_PROJECT_SERVICE_DIR}
COPY dn_entrypoint.init.bash dn_entrypoint.attach.bash dn_ros2_rebuild_dev_workspace.bash dn_fetch_ros_env_variables.bash dn_expose_container_env_variables.bash ./

RUN chmod +x dn_entrypoint.init.bash && \
    chmod +x dn_entrypoint.attach.bash && \
    chmod +x dn_ros2_rebuild_dev_workspace.bash && \
    chmod +x dn_fetch_ros_env_variables.bash && \
    chmod +x dn_expose_container_env_variables.bash

# Create directories
ENV DN_PROJECT_USER_HOME=/home/${DN_PROJECT_USER}
WORKDIR ${DN_PROJECT_USER_HOME}
WORKDIR /dn_container_env_variable/

# Create soft link in the home dir
RUN ln -s /dn_container_env_variable ${DN_PROJECT_USER_HOME}/dn_container_env_variable \
    && ln -s ${DN_PROJECT_SERVICE_DIR}/dn_entrypoint.init.bash ${DN_PROJECT_USER_HOME}/dn_entrypoint.init.bash \
    && ln -s ${DN_PROJECT_SERVICE_DIR}/dn_entrypoint.attach.bash ${DN_PROJECT_USER_HOME}/dn_entrypoint.attach.bash \
    && ln -s ${DN_PROJECT_SERVICE_DIR}/dn_ros2_rebuild_dev_workspace.bash ${DN_PROJECT_USER_HOME}/dn_ros2_rebuild_dev_workspace.bash \
    && ln -s ${DN_PROJECT_SERVICE_DIR}/dn_fetch_ros_env_variables.bash ${DN_PROJECT_USER_HOME}/dn_fetch_ros_env_variables.bash \
    && ln -s ${DN_PROJECT_SERVICE_DIR}/dn_expose_container_env_variables.bash ${DN_PROJECT_USER_HOME}/dn_expose_container_env_variables.bash

# ....Project directories ownership and simlink....................................................

# (CRITICAL) ToDo: validate user have permission to execute file in that dir (ref task NMO-548)
RUN chown -R $(id -u ${DN_PROJECT_USER}):$(id -g ${DN_PROJECT_USER}) ${DN_PROJECT_USER_HOME}
RUN chown -R $(id -u ${DN_PROJECT_USER}):$(id -g ${DN_PROJECT_USER}) ${DN_PROJECT_PATH}
RUN chown -R $(id -u ${DN_PROJECT_USER}):$(id -g ${DN_PROJECT_USER}) ${DN_DEV_WORKSPACE}

# (CRITICAL) ToDo: Not sure its usefull to symlink those (ref task NMO-548)
RUN ln -s "${DN_DEV_WORKSPACE}" "${DN_PROJECT_USER_HOME}${DN_DEV_WORKSPACE}" && \
    ln -s "${DN_PROJECT_PATH}" "${DN_PROJECT_USER_HOME}${DN_PROJECT_PATH}"

FROM project-specific-ros-setup AS test-ssh-user
USER "${DN_SSH_SERVER_USER}"
SHELL ["/bin/bash", "-c"]
WORKDIR /

RUN test $(whoami) == "${DN_SSH_SERVER_USER}" && test -d /home/${DN_SSH_SERVER_USER} || exit 1

RUN echo "===Check that ROS source is available via the ssh daemon user====================" \
    && printenv \
    && echo "" \
    && echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" \
    && python -c "import rclpy" \
    && echo "" \
    && echo "===ros2 install pkg sanity check=================================================" \
    && { \
        test -n "$(ros2 pkg list | grep -e ros_core)" &&  \
        test -n "$(ros2 pkg list | grep -e rclpy)" &&  \
        test -n "$(ros2 pkg list | grep -e ackermann_msgs)" &&  \
        test -n "$(ros2 pkg list | grep -e vesc_msgs)" ; \
    }||  exit 1

FROM project-specific-ros-setup AS test-project-user
USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/bin/bash", "-c"]
ARG TARGETPLATFORM
ARG BUILDPLATFORM
WORKDIR /

RUN test $(whoami) == "${DN_PROJECT_USER}" && test -d /home/${DN_PROJECT_USER} || exit 1

RUN echo "===Check that ROS source is available in DN_PROJECT_USER ssh daemon user=============" \
    && printenv \
    && echo "" \
    && echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" \
    && python -c "import rclpy" \
    && echo "" \
    && echo "===ros2 install pkg sanity check=================================================" \
    && { \
        test -n "$(ros2 pkg list | grep -e ros_core)" &&  \
        test -n "$(ros2 pkg list | grep -e rclpy)" &&  \
        test -n "$(ros2 pkg list | grep -e ackermann_msgs)" &&  \
        test -n "$(ros2 pkg list | grep -e vesc_msgs)" ; \
    } ||  exit 1

WORKDIR ${DN_PROJECT_USER_HOME}
RUN { \
        test -x "${DN_PROJECT_USER_HOME}/dn_expose_container_env_variables.bash" && \
        bash dn_expose_container_env_variables.bash && \
        TMP_DOTENV_EXPOSE_PATH="/dn_container_env_variable/.env.dn_expose_${DN_CONTAINER_NAME}" && \
        test -f "${TMP_DOTENV_EXPOSE_PATH}" && \
        TMP_DOTENV_EXPOSE_USER_PATH="${DN_PROJECT_USER_HOME}/dn_container_env_variable/.env.dn_expose_${DN_CONTAINER_NAME}" && \
        test -f "${TMP_DOTENV_EXPOSE_USER_PATH}" ; \
    } || exit 1

RUN { \
        test -x "${DN_PROJECT_USER_HOME}/dn_ros2_rebuild_dev_workspace.bash" && \
        if [[ ${TARGETPLATFORM:?err} != ${BUILDPLATFORM:?err} ]]; then \
          bash ./dn_ros2_rebuild_dev_workspace.bash 'virtualization' ; \
        else \
          bash ./dn_ros2_rebuild_dev_workspace.bash 'native' ; \
        fi ; \
    } || exit 1

RUN { \
    test -x "${DN_PROJECT_USER_HOME}/dn_entrypoint.init.bash" && \
    test -x "${DN_PROJECT_USER_HOME}/dn_entrypoint.attach.bash" && \
    test -d "${DN_PROJECT_USER_HOME}" && \
    test -d "${DN_PROJECT_PATH}" && \
    test -d "${DN_DEV_WORKSPACE}" ; \
    } || exit 1


RUN { \
    test -x "${DN_PROJECT_USER_HOME}/dn_fetch_ros_env_variables.bash" && \
    bash ./dn_fetch_ros_env_variables.bash ; \
    } || exit 1

# unset entrypoint for running test stage
ENTRYPOINT [ "" ]

FROM project-specific-ros-setup AS final

USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/bin/bash", "-c"]

#WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
WORKDIR ${DN_PROJECT_USER_HOME:?'environment variable is not set'}
ENTRYPOINT [ "/dockerized-norlab/project/project-develop/dn_entrypoint.init.bash" ]
CMD [ "bash" ]




