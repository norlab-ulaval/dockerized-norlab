
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS debugging-tools

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
    && rm ./dn_install_debugging_tools.bash


FROM debugging-tools AS package-project-src-code

ARG DN_DEPLOY_REPO_BRANCH=main

## (Priority) todo:implement >> everything ↓↓
#RUN echo "${0}:not implemented error" && exit

WORKDIR ${DN_DEV_WORKSPACE}/src
RUN echo "Sanity check"\
    && echo ${DN_PROJECT_GIT_NAME:?'Build argument needs to be set and non-empty.'} \
    && echo ${DN_PROJECT_GIT_DOMAIN:?'Build argument needs to be set and non-empty.'} \
    && echo ${DN_DEPLOY_REPO_BRANCH:?'Build argument needs to be set and non-empty.'} \
    && echo \
    && echo "Pull latest ${DN_DEPLOY_REPO_BRANCH} branch" \
    && git clone --branch ${DN_DEPLOY_REPO_BRANCH} https://github.com/${DN_PROJECT_GIT_DOMAIN}/${DN_PROJECT_GIT_NAME}.git  \
    && cd ${DN_DEV_WORKSPACE} \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --default-yes \
    && rm -rf /var/lib/apt/lists/*


WORKDIR ${DN_DEV_WORKSPACE}


# (CRITICAL) ToDo: validate >> source /opt/ros/${ROS_DISTRO}/setup.bash VS source /opt/ros/${ROS_DISTRO}/install/setup.bash
#RUN source /opt/ros/${ROS_DISTRO}/install/setup.bash \
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install --event-handlers console_direct+


FROM package-project-src-code AS test

USER "${DN_SSH_SERVER_USER}"

# Note: The .bashrc files get sourced only for interactive shell, so use 'bash -i' when
#       dockerfile build stage require to source ~/.bashrc.
#       see https://stackoverflow.com/a/74017557 by Chuck Batson
SHELL ["/bin/bash", "-i", "-c"]

RUN echo "Check that ROS source is available via the ssh daemon user" \
    && [[ $(whoami) == ${DN_SSH_SERVER_USER}  ]] \
    && rosversion -d \
    && printenv \
    && echo ${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'} \
    && python -c "import rclpy"

ENTRYPOINT [ "" ]

FROM package-project-src-code AS final

WORKDIR /

COPY dn_ros2_entrypoint.bash /
RUN chmod +x /dn_ros2_entrypoint.bash

ENTRYPOINT [ "/dn_ros2_entrypoint.bash" ]
CMD [ "bash" ]




