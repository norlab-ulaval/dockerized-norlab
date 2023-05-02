
# /// norlab-ros2/develop //////////////////////////////////////////////////////////////////////////////////////////////

#ARG  BASE_IMG_TAG=arm64-l4t-r32.6.1
ARG  BASE_IMG_TAG=arm64-l4t-r35.2.1
#FROM norlabsnow/norlab-ros2-humble-dependencies:${BASE_IMG_TAG}
FROM norlabsnow/project_f1tenth:${BASE_IMG_TAG}



# Default: Local , Option: XavierWarthog
ARG DS_HOST_TYPE=Local
ENV DS_HOST_TYPE=${DS_HOST_TYPE}

# ... Finish container setup ...........................................................................................
WORKDIR /
COPY ros2_rebuild_dev_workspace.bash /
COPY ros2_entrypoint.bash /
COPY fetch_ros_env.bash /
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros2_entrypoint.bash" \
    && /bin/bash -c "chmod +x /ros2_rebuild_dev_workspace.bash" \
    && /bin/bash -c "chmod +x /fetch_ros_env.bash"


RUN ( \
  echo ""; \
  echo "# dockerized-norlab aliases (from develop img)"; \
  echo "alias dn_ros2_rebuild_dev_workspace='bash /ros2_rebuild_dev_workspace.bash'"; \
  echo "alias dn_fetch_ros_env='bash /fetch_ros_env.bash'"; \
  echo ""; \
) >> ~/.bashrc

WORKDIR /ros2_ws/src/${DN_TARGET_PROJECT_SRC_REPO}
ENTRYPOINT [ "/ros2_entrypoint.bash" ]
CMD [ "bash" ]

# ////////////////////////////////////////////////////////////////////////////////////////////// norlab-ros2/develop ///



