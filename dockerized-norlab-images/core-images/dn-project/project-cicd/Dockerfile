
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS norlab-teamcity-server-setup


SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive


RUN ${DN_PROJECT_GIT_NAME:?'Build argument needs to be set and non-empty.'} \
    && echo ${DN_PROJECT_GIT_DOMAIN:?'Build argument needs to be set and non-empty.'}


WORKDIR /

# ===install teamcity & CI/CD utilities==================================================================

RUN pip3 install --no-cache-dir --verbose --upgrade pip
RUN pip3 install --no-cache-dir --verbose --upgrade pytest
RUN pip3 install --no-cache-dir --verbose teamcity-messages

RUN ( \
  echo "Configure Matplotlib to use a non-interactive backends for TeamCity run"; \
  echo "backend : Agg" >> /etc/matplotlibrc; \
)


# ===Final build step========================================================================
#WORKDIR /
#RUN echo $PWD
#COPY ./container-tools/dn_info.bash /
#RUN cd / && tree -L 3


# ...source & check ROS environement variable...............................................
# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
# Check the ROS_PACKAGE_PATH environment variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser${DN_DEV_WORKSPACE}/src:/opt/ros/melodic/share
WORKDIR ${DN_DEV_WORKSPACE}

# (CRITICAL) ToDo: validate >> source /opt/ros/${ROS_DISTRO}/setup.bash VS source /opt/ros/${ROS_DISTRO}/install/setup.bash
#
#RUN source /opt/ros/${ROS_DISTRO}/install/setup.bash \
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source ${DN_DEV_WORKSPACE}/install/setup.bash \
    && printenv | grep -e AR_ -e ROS -e MASTER -e HOSTNAME -e DS_



# ... Finish container setup ............................................................
WORKDIR ${DN_DEV_WORKSPACE}/src/${DN_PROJECT_GIT_NAME}
CMD [ "bash" ]
