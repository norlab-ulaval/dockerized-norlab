
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS build-norlab-perception-ros-stack

#ARG ROS_DISTRO
#ENV ROS_DISTRO=${ROS_DISTRO:?'Environment variable was not passed from base-image build stage'}

# ====Requirement==================================================================================
WORKDIR ${DN_DEV_WORKSPACE}

RUN apt-get update --fix-missing \
    && apt-get install --assume-yes \
          ros-${ROS_DISTRO:?err}-geometry2 \
          ros-${ROS_DISTRO}-tf2 \
          ros-${ROS_DISTRO}-tf2-ros \
          ros-${ROS_DISTRO}-tf2-tools \
          ros-${ROS_DISTRO}-tf2-eigen \
          ros-${ROS_DISTRO}-nav-msgs \
          ros-${ROS_DISTRO}-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*



# ====Begin========================================================================================
# Reference: https://github.com/norlab-ulaval/Norlab_wiki/wiki/Tutorials:-3D-Mapping-(ros2)

##ENV CMAKE_INCLUDE_PATH=${PERCEP3D_LIB_PATH}/include
#ENV CMAKE_PREFIX_PATH="${PERCEP3D_LIB_PATH}:${PERCEP3D_LIB_PATH}/share"
#ENV CMAKE_INSTALL_PREFIX=${PERCEP3D_LIB_PATH}
##ENV libpointmatcher_INCLUDE_DIRS=${PERCEP3D_LIB_PATH}/libpointmatcher
#ENV libpointmatcher_DIR="${PERCEP3D_LIB_PATH}/share/libpointmatcher/cmake"
## ★★★ Note: Consider something around the line of libpointmatcher_DIR=${PERCEP3D_LIB_PATH}/libpointmatcher/build/install/share/libpointmatcher/cmake



# ....fetch libpointmatcher_ros....................................................................
# (NICE TO HAVE) ToDo: move libpointmatcher_ros install step to dn-perception-libpointmatcher image
WORKDIR ${DN_DEV_WORKSPACE:?'Variable not set'}/src/
RUN git clone --branch ${ROS_DISTRO:?err} https://github.com/norlab-ulaval/libpointmatcher_ros.git

# ....fetch norlab_icp_mapper_ros..................................................................
WORKDIR ${DN_DEV_WORKSPACE}/src/
RUN git clone --branch ${ROS_DISTRO} https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git

# ....fetch mapper_config_template.................................................................
WORKDIR ${DN_DEV_WORKSPACE}/src/
RUN git clone https://github.com/norlab-ulaval/mapper_config_template.git

# ....Fetch demo data..............................................................................
#WORKDIR ${HOME}
WORKDIR ${DN_DEV_WORKSPACE}
RUN wget -O demo.zip "http://norlab.s3.valeria.science/demo.zip?AWSAccessKeyId=XMBLP3A0338XN5LASKV2&Expires=2290342833&Signature=NXn1f%2BeJ7NptrzUkNzHtEvdicjc%3D" \
    && unzip demo.zip \
    && rm demo.zip

# ....Build perception 3D ROS packages.............................................................

# (CRITICAL) ToDo: investigate?? >> check for adding the package path to CMAKE_PREFIX_PATH like libpointmatcher, libnabo and norlab-icp-mapper
#                   see https://www.notion.so/redleader962/Dockerized-NorLab-cff173c35eba49989eee9275d3613ce0?pvs=4#89f6364a3a9f46ffb4a268c6f80e0db6

WORKDIR ${DN_DEV_WORKSPACE}

# (Priority) ToDo: assess moving rosbag install earlier
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1 \
    && apt-get update --fix-missing \
    && apt-get install --assume-yes \
        ros-${ROS_DISTRO}-rosbag2 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ToDo: validate install steps in here -> https://github.com/norlab-ulaval/Norlab_wiki/wiki/Tutorials:-3D-Mapping-(ros2)
RUN echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash" \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && echo "sourcing ${DN_DEV_WORKSPACE}/install/setup.bash" \
    && source ${DN_DEV_WORKSPACE}/install/setup.bash \
    && apt-get update --fix-missing \
    && rosdep update --rosdistro ${ROS_DISTRO} --include-eol-distros \
    && rosdep fix-permissions \
    && rosdep install  \
            --ignore-packages-from-source \
            --from-path ./src  \
            --rosdistro ${ROS_DISTRO}  \
            -y \
    && colcon version-check \
    && colcon build \
            --symlink-install \
            --event-handlers console_direct+

#RUN echo "Check workspace directory installation" \
#    && [[ -f ${HOME}/demo/config.rviz ]] \
#    || exit 1

COPY ./entrypoint.norlab-stack.bash ./
RUN chmod +x entrypoint.norlab-stack.bash

FROM build-norlab-perception-ros-stack as test

WORKDIR ${DN_DEV_WORKSPACE}

RUN echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash" \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && echo "sourcing ${DN_DEV_WORKSPACE}/install/setup.bash" \
    && source ${DN_DEV_WORKSPACE}/install/setup.bash \
    && ros2 bag play ${DN_DEV_WORKSPACE}/demo/demo

#    && ros2 launch mapper_config_template mapper.launch \

FROM build-norlab-perception-ros-stack as final

# (NICE TO HAVE) ToDo: implement entrypoint with demo example runing

WORKDIR ${DN_DEV_WORKSPACE}

ENTRYPOINT [ "./entrypoint.norlab-stack.bash" ]
CMD [ "bash" ]

