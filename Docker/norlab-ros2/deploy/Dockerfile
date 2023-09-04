
# /// norlab-ros2/deploy ///////////////////////////////////////////////////////////////////////////////////////////////

# norlabsnow/norlab-ros-dependencies:foxy-pytorch-arm64-l4t-r35.2.1
ARG DN_PROJECT_HUB=norlabsnow
ARG DN_PROJECT_IMAGE_NAME=norlab-ros-project-template
ARG DN_BASE_IMG_TAG=humble-pytorch-arm64-l4t-r35.2.1
FROM ${DN_PROJECT_HUB}/${DN_PROJECT_IMAGE_NAME}:${DN_BASE_IMG_TAG} as deploy

# === Clone the repo and install =======================================================================================
# Steps:
#   1. clone NorLab_MPPI repos
#   2. Install Dependencies
#   3. Compilation & Running
# =====================================================================================================================

#ARG DN_TARGET_PROJECT_SRC_REPO
#ENV DN_TARGET_PROJECT_SRC_REPO=${DN_TARGET_PROJECT_SRC_REPO:?'Build argument needs to be set and non-empty.'}
#ARG DN_TARGET_PROJECT_SRC_DOMAIN
#ENV DN_TARGET_PROJECT_SRC_DOMAIN=${DN_TARGET_PROJECT_SRC_DOMAIN:?'Build argument needs to be set and non-empty.'}

ARG DN_DEPLOY_REPO_BRANCH=master

# Default: Local , Option: XavierWarthog
ARG DS_HOST_TYPE=Local
ENV DS_HOST_TYPE=${DS_HOST_TYPE}

## (Priority) todo:implement >> everything ↓↓
#RUN echo "${0}:not implemented error" && exit

WORKDIR ${DN_DEV_WORKSPACE}/src
RUN echo "Pull latest ${DN_DEPLOY_REPO_BRANCH} branch" \
    && git clone --branch ${DN_DEPLOY_REPO_BRANCH} https://github.com/${DN_TARGET_PROJECT_SRC_DOMAIN}/${DN_TARGET_PROJECT_SRC_REPO}.git  \
    && cd ${DN_DEV_WORKSPACE} \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --default-yes \
    && rm -rf /var/lib/apt/lists/*


WORKDIR ${DN_DEV_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash \
        && colcon build --symlink-install --merge-install"

# (Priority) todo:refactor (ref task NLSAR-222 🛠→ setupEnv*.sh scripts for deployement case)
## Environment setup
#RUN echo ". ${DN_DEV_WORKSPACE}/src/${DN_TARGET_PROJECT_SRC_REPO}/mppi_util/setupEnv${DS_HOST_TYPE}.sh" >> ~/.bashrc


# ... Finish container setup ...........................................................................................
WORKDIR /
COPY ros_entrypoint.bash /
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros_entrypoint.bash"
ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD [ "bash" ]

# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS

# /////////////////////////////////////////////////////////////////////////////////////////////// norlab-ros2/deploy ///



