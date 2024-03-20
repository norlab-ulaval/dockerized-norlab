
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
    && apt-get update --fix-missing \
    && rosdep update --rosdistro ${ROS_DISTRO} --include-eol-distros \
    && rosdep fix-permissions \
    && rosdep install  \
            --ignore-packages-from-source \
            --from-path ./src  \
            --rosdistro ${ROS_DISTRO}  \
            -y \
    && rm -rf /var/lib/apt/lists/*


WORKDIR ${DN_DEV_WORKSPACE}

ARG TARGETPLATFORM
ARG BUILDPLATFORM
RUN echo "sourcing ${DN_DEV_WORKSPACE}/install/setup.bash" \
    && source ${DN_DEV_WORKSPACE}/install/setup.bash \
    && echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash" \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && COLCON_FLAGS=() \
    && if [[ ${TARGETPLATFORM:?err} != ${BUILDPLATFORM:?err} ]]; then \
         echo -e "Builder is running in architecture virtualisation" \
         && COLCON_FLAGS+=(--executor sequential) ; \
      else \
         echo -e "Builder is running on native architecture" \
         && COLCON_FLAGS+=(--symlink-install) ; \
      fi \
    && COLCON_FLAGS+=( \
          --cmake-clean-cache \
          --cmake-args -DCMAKE_BUILD_TYPE=Release \
          --event-handlers console_direct+ \
       ) \
    && echo -e "COLCON_FLAGS=("${COLCON_FLAGS[*]}")" \
    && colcon build ${COLCON_FLAGS[@]}


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

# unset entrypoint for running test stage
ENTRYPOINT [ "" ]

FROM package-project-src-code AS final

COPY dn_ros2_entrypoint.bash /
RUN chmod +x /dn_ros2_entrypoint.bash

WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
ENTRYPOINT [ "/dn_ros2_entrypoint.bash" ]
CMD [ "bash" ]




