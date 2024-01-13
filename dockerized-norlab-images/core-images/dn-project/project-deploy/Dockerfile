
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
WORKDIR ${DN_PATH}/dockerized-norlab-images/container-tools

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

RUN echo "sourcing /opt/ros/${ROS_DISTRO}/install/setup.bash" \
    && source /opt/ros/${ROS_DISTRO}/install/setup.bash \
    && echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash" \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install --event-handlers console_direct+


FROM package-project-src-code AS test

USER "${DN_SSH_SERVER_USER}"

SHELL ["/bin/bash", "-c"]

RUN echo "===Check that ROS source is available via the ssh daemon user====================" \
    && [[ $(whoami) == ${DN_SSH_SERVER_USER}  ]] \
    && printenv \
    && echo "" \
    && echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" \
    && python -c "import rclpy" \
    && echo "" \
    && echo "===ros2 install pkg sanity check=================================================" \
    && [[ -n "$(ros2 pkg list | grep -e ros_core)" ]] ||  exit 1  \
    && [[ -n "$(ros2 pkg list | grep -e rclpy)" ]] ||  exit 1  \
    && [[ -n "$(ros2 pkg list | grep -e ackermann_msgs)" ]] ||  exit 1  \
    && [[ -n "$(ros2 pkg list | grep -e vesc_msgs)" ]] ||  exit 1

ENTRYPOINT [ "" ]

FROM package-project-src-code AS final

COPY dn_ros2_entrypoint.bash /
RUN chmod +x /dn_ros2_entrypoint.bash

WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
ENTRYPOINT [ "/dn_ros2_entrypoint.bash" ]
CMD [ "bash" ]




