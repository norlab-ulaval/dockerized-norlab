
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
COPY dn_ros2_rebuild_dev_workspace.bash /
COPY dn_ros2_entrypoint.bash /
COPY dn_fetch_ros_env.bash /

# (CRITICAL) ToDo: test using pytest-dotenv in pycharm (ref task NMO-360 ﹅→ Fix the ROS environment variable not loaded in PyCharm pytest run nightmare)
# https://github.com/quiqua/pytest-dotenv
# How to get PyCharm test run working with pytest-dotenv?:  https://github.com/quiqua/pytest-dotenv/issues/10
RUN pip3 install --no-cache-dir --verbose --upgrade pip  \
    && pip3 install --no-cache-dir --verbose  \
        pytest-dotenv

## https://github.com/abrahammurciano/python-shell-source
#RUN pip3 install --no-cache-dir --verbose --upgrade pip  \
#    && pip3 install --no-cache-dir --verbose  \
#        shell-source

# set read/write permission to entrypoint file and joystick dir js0
RUN chmod +x /dn_ros2_entrypoint.bash \
    && chmod +x /dn_ros2_rebuild_dev_workspace.bash \
    && chmod +x /dn_fetch_ros_env.bash \
    && ( \
          echo ""; \
          echo "# >>> dockerized-norlab aliases (from develop img)"; \
          echo "alias dn_ros2_rebuild_dev_workspace='source /dn_ros2_rebuild_dev_workspace.bash'"; \
          echo "alias dn_fetch_ros_env='source /dn_fetch_ros_env.bash'"; \
          echo ""; \
        ) >> ~/.bashrc

FROM project-specific-ros-setup AS test

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

FROM project-specific-ros-setup AS final

WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
ENTRYPOINT [ "/dn_ros2_entrypoint.bash" ]
CMD [ "bash" ]




