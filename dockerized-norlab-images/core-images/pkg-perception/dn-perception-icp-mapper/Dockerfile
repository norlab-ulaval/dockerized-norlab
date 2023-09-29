
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS icp-mapper-install


# ====Begin=======================================================================================
# Reference: https://github.com/norlab-ulaval/Norlab_wiki/wiki/Tutorials:-3D-Mapping-(ros2)

##ENV CMAKE_INCLUDE_PATH=${PERCEP3D_LIB_PATH}/include
#ENV CMAKE_PREFIX_PATH="${PERCEP3D_LIB_PATH}:${PERCEP3D_LIB_PATH}/share"
#ENV CMAKE_INSTALL_PREFIX=${PERCEP3D_LIB_PATH}
##ENV libpointmatcher_INCLUDE_DIRS=${PERCEP3D_LIB_PATH}/libpointmatcher
#ENV libpointmatcher_DIR="${PERCEP3D_LIB_PATH}/share/libpointmatcher/cmake"
## ★★★ Note: Consider something around the line of libpointmatcher_DIR=${PERCEP3D_LIB_PATH}/libpointmatcher/build/install/share/libpointmatcher/cmake

# ....Install norlab_icp_mapper...................................................................
WORKDIR ${PERCEP3D_LIB_PATH:?'Error: env variable empty'}
#
RUN bash -c "[[ -d "${PERCEP3D_LIB_PATH}/libpointmatcher" ]] | echo \"libpointmatcher dir is unreachable\"" \
    && git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
    && cd norlab_icp_mapper/ \
    && mkdir build && cd build/ \
    && cmake -D CMAKE_BUILD_TYPE=Release \
             -D CMAKE_INSTALL_PREFIX=${DN_CMAKE_INSTALL_PREFIX} \
             .. \
    && make -j $(nproc) \
    && make install
#             -D libpointmatcher_INCLUDE_DIRS=${libpointmatcher_INCLUDE_DIRS} \

#ENV norlab_icp_mapper_DIR=${DN_CMAKE_INSTALL_PREFIX}/share/norlab_icp_mapper/cmake

FROM icp-mapper-install AS build-ros-packages

# ....fetch libpointmatcher_ros....................................................................
# (NICE TO HAVE) ToDo: move libpointmatcher_ros install step to dn-perception-libpointmatcher image
WORKDIR ${DN_DEV_WORKSPACE}/src/
RUN git clone --branch ${ROS_DISTRO} https://github.com/norlab-ulaval/libpointmatcher_ros.git

# ....fetch norlab_icp_mapper_ros..................................................................
WORKDIR ${DN_DEV_WORKSPACE}/src/
RUN git clone --branch ${ROS_DISTRO} https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git

# ....fetch mapper_config_template.................................................................
WORKDIR ${DN_DEV_WORKSPACE}/src/
RUN git clone https://github.com/norlab-ulaval/mapper_config_template.git


# ....Build perception 3D ROS packages.............................................................

# (CRITICAL) ToDo: investigate?? >> check for adding the package path to CMAKE_PREFIX_PATH like libpointmatcher, libnabo and norlab-icp-mapper
#                   see https://www.notion.so/redleader962/Dockerized-NorLab-cff173c35eba49989eee9275d3613ce0?pvs=4#89f6364a3a9f46ffb4a268c6f80e0db6

#WORKDIR ${PERCEP3D_LIB_PATH:?'Error › env variable empty'}
WORKDIR ${DN_DEV_WORKSPACE}
RUN echo "sourcing /opt/ros/${ROS_DISTRO}/install/setup.bash" \
    && source /opt/ros/${ROS_DISTRO}/install/setup.bash \
    && echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash" \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install \
        --cmake-args " -D CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${DN_CMAKE_INSTALL_PREFIX}"


#        --cmake-args " -D CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${DN_CMAKE_INSTALL_PREFIX}/include/norlab_icp_mapper/build:${DN_CMAKE_INSTALL_PREFIX}/share/libnabo/cmake:${DN_CMAKE_INSTALL_PREFIX}/share/libpointmatcher/cmake"

#        --cmake-args " -D CMAKE_INSTALL_PREFIX=${DN_CMAKE_INSTALL_PREFIX}"
#    --cmake-args " -D libpointmatcher_DIR=${libpointmatcher_DIR:?'Error: env variable empty'}"


#    && source ${DN_DEV_WORKSPACE}/install/setup.bash
#
##FROM build-ros-packages AS test
##
##WORKDIR ~/
##RUN wget -O demo.zip "http://norlab.s3.valeria.science/demo.zip?AWSAccessKeyId=XMBLP3A0338XN5LASKV2&Expires=2290342833&Signature=NXn1f%2BeJ7NptrzUkNzHtEvdicjc%3D" \
##    && unzip demo.zip \
##    && rm demo.zip
##
##RUN echo "sourcing /opt/ros/${ROS_DISTRO}/install/setup.bash" \
##    && source /opt/ros/${ROS_DISTRO}/install/setup.bash \
##    && echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash" \
##    && source /opt/ros/${ROS_DISTRO}/setup.bash \
##    && source ${DN_DEV_WORKSPACE}/install/setup.bash \
##    && ros2 launch mapper_config_template mapper.launch \
##    && ros2 bag play ~/demo/demo --clock
##
##FROM build-ros-packages AS final
##
##WORKDIR ${DN_DEV_WORKSPACE}
##CMD [ "bash" ]
##
