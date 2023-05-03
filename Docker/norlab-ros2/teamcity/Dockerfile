
# /// norlab-ros2/teamcity /////////////////////////////////////////////////////////////////////////////////////////////

#ARG DN_BASE_IMG_TAG=arm64-l4t-r32.6.1
ARG DN_BASE_IMG_TAG=humble-pytorch-arm64-l4t-r35.2.1
#ARG BASE_IMGAGE=norlab-ros-dependencies-wo-services
ARG BASE_IMGAGE=norlab-ros-project-template
FROM norlabsnow/${BASE_IMGAGE}:${DN_BASE_IMG_TAG} as teamcity


ARG DN_TARGET_PROJECT_SRC_REPO=NorLab_MPPI
ENV DN_TARGET_PROJECT_SRC_REPO=${DN_TARGET_PROJECT_SRC_REPO}


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
#   > /home/youruser/ros2_ws/src:/opt/ros/melodic/share
WORKDIR ${DS_DEV_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash \
    && source ${DS_DEV_WORKSPACE}/install/setup.bash \
    && printenv | grep -e AR_ -e ROS -e MASTER -e HOSTNAME -e DS_"



# ... Finish container setup ............................................................
WORKDIR /ros2_ws/src/${DN_TARGET_PROJECT_SRC_REPO}
CMD [ "bash" ]

# ///////////////////////////////////////////////////////////////////////////////////////////// norlab-ros2/teamcity ///



