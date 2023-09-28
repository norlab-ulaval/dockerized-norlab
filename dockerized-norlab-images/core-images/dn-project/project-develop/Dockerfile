
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

## ToDo: move the file upper in dn-project/ dir and add copy step to project-deploy also
## Execute ros sourcing system wide instead of in .bashrc (which is not executed in non-interactive shell)
#COPY 02-dn-source-ros-system-wide.sh /etc/profile.d
#
RUN touch /etc/profile.d/02-dn-source-ros-system-wide.sh \
    && chmod +x /etc/profile.d/02-dn-source-ros-system-wide.sh \
    && ( \
      echo ""; \
      echo "# Dockerized-NorLab › Execute ros sourcing system wide instead of in .bashrc which is not executed"; \
      echo "# in non-interactive shell such as in pycharm-debugger"; \
      echo "echo \"sourcing /opt/ros/${ROS_DISTRO}/install/setup.bash\""; \
      echo "source /opt/ros/${ROS_DISTRO}/setup.bash"; \
      echo ""; \
      echo "echo \"ros2 install pkg sanity check\""; \
      echo "[[ -n \"$(ros2 pkg list | grep -e ros_core)\" ]] || echo \"Can't find ros_core localy\" && exit "; \
      echo "[[ -n \"$(ros2 pkg list | grep -e rclpy)\" ]] || echo \"Can't find rclpy localy\" && exit "; \
      echo "[[ -n \"$(ros2 pkg list | grep -e ackermann_msgs)\" ]] || echo \"Can't find ackermann_msgs localy\" && exit "; \
      echo "[[ -n \"$(ros2 pkg list | grep -e vesc_msgs)\" ]] || echo \"Can't find vesc_msgs localy\" && exit "; \
      echo ""; \
      echo "# Dockerized-NorLab › Quickhack for connecting to pycharm-debugger (over ssh)"; \
      echo "export DN_CONTAINER_NAME=$DN_CONTAINER_NAME"; \
      echo "export DN_ACTIVATE_POWERLINE_PROMT=$DN_ACTIVATE_POWERLINE_PROMT"; \
      echo ""; \
    ) > /etc/profile.d/02-dn-source-ros-system-wide.sh

#      echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash"; \

# To source ros in non-interactive & non-login shell
#RUN echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash" >> "$BASH_ENV"
ENV BASH_ENV=/etc/profile.d/02-dn-source-ros-system-wide.sh

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
          echo "alias dn_ros2_rebuild_dev_workspace='bash /dn_ros2_rebuild_dev_workspace.bash'"; \
          echo "alias dn_fetch_ros_env='bash /dn_fetch_ros_env.bash'"; \
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
ENTRYPOINT [ "/dn_ros2_entrypoint.bash" ]
CMD [ "bash" ]




