
# /// norlab-ros2/develop //////////////////////////////////////////////////////////////////////////////////////////////

#ARG  BASE_IMG_TAG=arm64-l4t-r32.6.1
ARG  BASE_IMG_TAG=arm64-l4t-r35.2.1
FROM norlabsnow/norlab-ros2-humble-dependencies:${BASE_IMG_TAG}

ARG DS_TARGET_PROJECT_SRC_REPO=NorLab_MPPI
# Note: `DS_TARGET_PROJECT_SRC_REPO` is required by the ros2_rebuild_norlab_mppi.bash script
ENV DS_TARGET_PROJECT_SRC_REPO=${DS_TARGET_PROJECT_SRC_REPO}

# Default: Local , Option: XavierWarthog
ARG DS_HOST_TYPE=Local
ENV DS_HOST_TYPE=${DS_HOST_TYPE}

# ... Finish container setup ...........................................................................................
WORKDIR /
COPY ros2_rebuild_norlab_mppi.bash /
COPY ros2_entrypoint.bash /
COPY fetch_ros_env.bash /
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros2_entrypoint.bash" \
    && /bin/bash -c "chmod +x /ros2_rebuild_norlab_mppi.bash" \
    && /bin/bash -c "chmod +x /fetch_ros_env.bash"


RUN ( \
  echo ""; \
  echo "# dockerized-norlab aliases (from develop img)"; \
  echo "alias ds_rebuild_norlab_mppi='bash /ros2_rebuild_norlab_mppi.bash'"; \
  echo "alias ds_fetch_ros_env='bash /fetch_ros_env.bash'"; \
  echo "alias ds_python3_check='python3 /ros2_ws/src/${DS_TARGET_PROJECT_SRC_REPO}/src/container_related/try_pytorch.py'"; \
  echo ""; \
) >> ~/.bashrc

WORKDIR /ros2_ws/src/${DS_TARGET_PROJECT_SRC_REPO}
ENTRYPOINT [ "/ros2_entrypoint.bash" ]
CMD [ "bash" ]

# ////////////////////////////////////////////////////////////////////////////////////////////// norlab-ros2/develop ///



