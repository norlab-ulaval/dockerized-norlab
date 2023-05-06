
# /// norlab-ros2/teamcity /////////////////////////////////////////////////////////////////////////////////////////////

# norlabsnow/norlab-ros-dependencies-wo-services:foxy-pytorch-arm64-l4t-r35.2.1
ARG DN_PROJECT_HUB=norlabsnow
ARG DN_PROJECT_IMAGE_NAME=norlab-ros-project-template
ARG DN_BASE_IMG_TAG=humble-pytorch-arm64-l4t-r35.2.1
FROM ${DN_PROJECT_HUB}/${DN_PROJECT_IMAGE_NAME}:${DN_BASE_IMG_TAG} as teamcity


#ARG DN_TARGET_PROJECT_SRC_REPO
#ENV DN_TARGET_PROJECT_SRC_REPO=${DN_TARGET_PROJECT_SRC_REPO?err}
#ARG DN_TARGET_PROJECT_SRC_DOMAIN
#ENV DN_TARGET_PROJECT_SRC_DOMAIN=${DN_TARGET_PROJECT_SRC_DOMAIN?err}


WORKDIR /

# ===install teamcity & CI/CD utilities=================================================================================

RUN pip3 install --no-cache-dir --verbose --upgrade pip
RUN pip3 install --no-cache-dir --verbose --upgrade pytest
RUN pip3 install --no-cache-dir --verbose teamcity-messages

RUN ( \
  echo "Configure Matplotlib to use a non-interactive backends for TeamCity run"; \
  echo "backend : Agg" >> /etc/matplotlibrc; \
)


# ===Final build step=======================================================================================
#WORKDIR /
#RUN echo $PWD
#COPY ./tools/dn_info.bash /
#RUN cd / && tree -L 3


# ...source & check ROS environement variable...............................................
# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
# Check the ROS_PACKAGE_PATH environment variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser${DN_DEV_WORKSPACE}/src:/opt/ros/melodic/share
WORKDIR ${DN_DEV_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash \
    && source ${DN_DEV_WORKSPACE}/install/setup.bash \
    && printenv | grep -e AR_ -e ROS -e MASTER -e HOSTNAME -e DS_"



# ... Finish container setup ............................................................
WORKDIR ${DN_DEV_WORKSPACE}/src/${DN_TARGET_PROJECT_SRC_REPO}
CMD [ "bash" ]

# ///////////////////////////////////////////////////////////////////////////////////////////// norlab-ros2/teamcity ///



