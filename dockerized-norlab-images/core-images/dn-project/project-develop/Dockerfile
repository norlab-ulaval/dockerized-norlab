
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


FROM debugging-tools AS project-specific-ros-setup

WORKDIR /
COPY ros2_rebuild_dev_workspace.bash /
COPY ros2_entrypoint.bash /
COPY fetch_ros_env.bash /

# set read/write permission to entrypoint file and joystick dir js0
RUN chmod +x /ros2_entrypoint.bash \
    && chmod +x /ros2_rebuild_dev_workspace.bash \
    && chmod +x /fetch_ros_env.bash \
    && ( \
          echo ""; \
          echo "# >>> dockerized-norlab aliases (from develop img)"; \
          echo "alias dn_ros2_rebuild_dev_workspace='bash /ros2_rebuild_dev_workspace.bash'"; \
          echo "alias dn_fetch_ros_env='bash /fetch_ros_env.bash'"; \
          echo ""; \
        ) >> ~/.bashrc

FROM project-specific-ros-setup AS test

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

FROM project-specific-ros-setup AS final

WORKDIR ${DN_DEV_WORKSPACE}/src/${DN_PROJECT_GIT_NAME}
ENTRYPOINT [ "/ros2_entrypoint.bash" ]
CMD [ "bash" ]



