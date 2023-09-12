
# /// core-images/project-develop //////////////////////////////////////////////////////////////////////////////////////////////

# norlabsnow/norlab-ros-dependencies:foxy-pytorch-arm64-l4t-r35.2.1
ARG DN_PROJECT_HUB=norlabsnow
ARG DN_PROJECT_IMAGE_NAME=dockerized-norlab-project-template
ARG DN_BASE_IMG_TAG=humble-pytorch-arm64-l4t-r35.2.1
FROM ${DN_PROJECT_HUB}/${DN_PROJECT_IMAGE_NAME}:${DN_BASE_IMG_TAG} as develop


#ARG DN_TARGET_PROJECT_SRC_REPO
#ENV DN_TARGET_PROJECT_SRC_REPO=${DN_TARGET_PROJECT_SRC_REPO:?'Build argument needs to be set and non-empty.'}
#ARG DN_TARGET_PROJECT_SRC_DOMAIN
#ENV DN_TARGET_PROJECT_SRC_DOMAIN=${DN_TARGET_PROJECT_SRC_DOMAIN:?'Build argument needs to be set and non-empty.'}


# Default: Local , Option: XavierWarthog
ARG DS_HOST_TYPE=Local
ENV DS_HOST_TYPE=${DS_HOST_TYPE}

# ... Finish container setup ............................................................................
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

WORKDIR ${DN_DEV_WORKSPACE}/src/${DN_TARGET_PROJECT_SRC_REPO}
ENTRYPOINT [ "/ros2_entrypoint.bash" ]
CMD [ "bash" ]

# ////////////////////////////////////////////////////////////////////////////////////////////// core-images/project-develop ///



