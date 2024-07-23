
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS debugging-tools

RUN echo "Pre-condition checks" && { \
      test $(whoami) == "root" &&  \
      test -n ${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'} && \
      test -d /home/${DN_PROJECT_USER} ; \
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
    && rm ./dn_install_debugging_tools.bash \
    && test -f /etc/ssh/sshd_config_dockerized_norlab_openssh_server || exit 1

# Create the remote interpreter directory
WORKDIR /home/${DN_SSH_SERVER_USER}/tmp/${DN_PROJECT_GIT_NAME:?err}
RUN chown -R ${DN_SSH_SERVER_USER} /home/${DN_SSH_SERVER_USER}/tmp/${DN_PROJECT_GIT_NAME}/


# ====Project specific ros setup===================================================================
FROM debugging-tools AS project-specific-ros-setup

# (Priority) ToDo: refactor the rest of the stage as a standalone script that can be executed by project-deploy

# Note: Its easier and more robust to copy DN files to root and then symlink them in project user.
ENV DN_PROJECT_DEVELOP_DIR=/dockerized-norlab/project/project-develop
WORKDIR ${DN_PROJECT_DEVELOP_DIR}
COPY ./dn_ros2_entrypoint.bash ./
COPY ./dn_ros2_rebuild_dev_workspace.bash ./
COPY ./dn_fetch_ros_env_variables.bash ./
COPY ./dn_expose_container_env_variables.bash ./

RUN chmod +x ./dn_ros2_entrypoint.bash \
    && chmod +x ./dn_ros2_rebuild_dev_workspace.bash \
    && chmod +x ./dn_fetch_ros_env_variables.bash \
    && chmod +x ./dn_expose_container_env_variables.bash

# Create directories
ENV DN_PROJECT_USER_HOME=/home/${DN_PROJECT_USER}
WORKDIR /dn_container_env_variable/
RUN ln -s /dn_container_env_variable ${DN_PROJECT_USER_HOME}/dn_container_env_variable

# Create soft link in the home dir
RUN ln -s ${DN_PROJECT_DEVELOP_DIR}/dn_ros2_entrypoint.bash ${DN_PROJECT_USER_HOME}/dn_ros2_entrypoint.bash \
    && ln -s ${DN_PROJECT_DEVELOP_DIR}/dn_ros2_rebuild_dev_workspace.bash ${DN_PROJECT_USER_HOME}/dn_ros2_rebuild_dev_workspace.bash \
    && ln -s ${DN_PROJECT_DEVELOP_DIR}/dn_fetch_ros_env_variables.bash ${DN_PROJECT_USER_HOME}/dn_fetch_ros_env_variables.bash \
    && ln -s ${DN_PROJECT_DEVELOP_DIR}/dn_expose_container_env_variables.bash ${DN_PROJECT_USER_HOME}/dn_expose_container_env_variables.bash


# (CRITICAL) ToDo: validate user have permission to execute file in that dir (ref task NMO-548)
RUN chown -R $(id -u ${DN_PROJECT_USER}):$(id -g ${DN_PROJECT_USER}) ${DN_PROJECT_USER_HOME}


FROM project-specific-ros-setup AS test
WORKDIR /

# ....Test ssh user................................................................................
USER "${DN_SSH_SERVER_USER}"
SHELL ["/bin/bash", "-c"]

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

# ....Test project user............................................................................
USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/bin/bash", "-c"]

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

ARG TARGETPLATFORM
ARG BUILDPLATFORM
RUN { \
        test -x "${DN_PROJECT_USER_HOME}/dn_ros2_rebuild_dev_workspace.bash" && \
        if [[ ${TARGETPLATFORM:?err} != ${BUILDPLATFORM:?err} ]]; then \
          bash ./dn_ros2_rebuild_dev_workspace.bash 'virtualization' ; \
        else \
          bash ./dn_ros2_rebuild_dev_workspace.bash 'native' ; \
        fi ; \
    } || exit 1

RUN test -x "${DN_PROJECT_USER_HOME}/dn_ros2_entrypoint.bash" && \
    bash ./dn_ros2_entrypoint.bash || exit 1

RUN test -x "${DN_PROJECT_USER_HOME}/dn_fetch_ros_env_variables.bash" && \
    bash ./dn_fetch_ros_env_variables.bash || exit 1

# unset entrypoint for running test stage
ENTRYPOINT [ "" ]

FROM project-specific-ros-setup AS final

USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/bin/bash", "-c"]

#WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
WORKDIR ${DN_PROJECT_USER_HOME:?'environment variable is not set'}
ENTRYPOINT [ "/dockerized-norlab/project/project-develop/dn_ros2_entrypoint.bash" ]
CMD [ "bash" ]




