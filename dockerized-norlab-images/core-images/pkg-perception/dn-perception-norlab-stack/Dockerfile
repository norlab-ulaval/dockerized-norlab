
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS build-norlab-perception-ros-stack

# (CRITICAL) ToDo: on task NMO-584 end >> delete next bloc ↓↓
ARG DEBIAN_FRONTEND=noninteractive

ARG PERCEP3D_LIB_PATH=/opt/percep3d_libraries
ENV PERCEP3D_LIB_PATH=${PERCEP3D_LIB_PATH}

# ====Requirement==================================================================================
WORKDIR ${DN_DEV_WORKSPACE:?'Variable not set'}/src/

RUN apt-get update --fix-missing \
    && apt-get install --assume-yes \
          ros-${ROS_DISTRO:?err}-geometry2 \
          ros-${ROS_DISTRO}-tf2 \
          ros-${ROS_DISTRO}-tf2-ros \
          ros-${ROS_DISTRO}-tf2-tools \
          ros-${ROS_DISTRO}-tf2-eigen \
          ros-${ROS_DISTRO}-nav-msgs \
          ros-${ROS_DISTRO}-sensor-msgs \
    && apt-get clean \
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
RUN git clone --branch ${ROS_DISTRO:?err} https://github.com/norlab-ulaval/libpointmatcher_ros.git

# ....fetch norlab_icp_mapper_ros..................................................................
RUN git clone --branch ${ROS_DISTRO} https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git

# ....fetch mapper_config_template.................................................................
RUN git clone https://github.com/norlab-ulaval/mapper_config_template.git

# ....Fetch demo data..............................................................................
WORKDIR ${DN_DEV_WORKSPACE}
RUN wget -O demo.zip "http://norlab.s3.valeria.science/demo.zip?AWSAccessKeyId=XMBLP3A0338XN5LASKV2&Expires=2290342833&Signature=NXn1f%2BeJ7NptrzUkNzHtEvdicjc%3D" \
    && unzip demo.zip \
    && rm -f demo.zip

# ....Build perception 3D ROS packages.............................................................

# (CRITICAL) ToDo: investigate?? >> check for adding the package path to CMAKE_PREFIX_PATH like libpointmatcher, libnabo and norlab-icp-mapper
#  see https://www.notion.so/redleader962/Dockerized-NorLab-cff173c35eba49989eee9275d3613ce0?pvs=4#89f6364a3a9f46ffb4a268c6f80e0db6


# (Priority) ToDo: assess moving rosbag install earlier
RUN apt-get update --fix-missing \
    && apt-get install --assume-yes \
        ros-${ROS_DISTRO}-rosbag2 \
        ros-dev-tools\
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ToDo: validate install steps in here -> https://github.com/norlab-ulaval/Norlab_wiki/wiki/Tutorials:-3D-Mapping-(ros2)
ARG TARGETPLATFORM
ARG BUILDPLATFORM
RUN <<EOF
#!/bin/bash

export CMAKE_PREFIX_PATH="${PERCEP3D_LIB_PATH:?err}:${CMAKE_PREFIX_PATH}"

echo && n2st::print_msg "Run rosdep install" \
    && apt-get update --fix-missing \
    && cd "${P3D_ROS_DEV_WORKSPACE}" \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep fix-permissions \
    && rosdep install \
        --ignore-packages-from-source \
        --from-path ./src  \
        --rosdistro "${ROS_DISTRO}"  \
        -y \
    && colcon version-check \
    || exit 1

# Note: The colcon flag "--cmake-clean-cache" reset the python3 env
COLCON_FLAGS=()
if [[ ${TARGETPLATFORM:?err} != ${BUILDPLATFORM:?err} ]]; then
    echo -e "Builder is running in architecture virtualisation"
    COLCON_FLAGS+=(--executor sequential)
else
    echo -e "Builder is running on native architecture"
    COLCON_FLAGS+=(--symlink-install)
fi
COLCON_FLAGS+=(
      "--cmake-clean-cache"
      "--cmake-args" "-DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}"
      "--cmake-args" "-DCMAKE_BUILD_TYPE=Release"
      "--event-handlers" "console_direct+"
   )
echo && n2st::print_msg "Run colcon build ${COLCON_FLAGS[*]}" \
    && cd "${P3D_ROS_DEV_WORKSPACE}" \
    && colcon build "${COLCON_FLAGS[@]}" \
    || exit 1

EOF

#RUN echo "Check workspace directory installation" \
#    && [[ -f ${HOME}/demo/config.rviz ]] \
#    || exit 1

COPY ./entrypoint.norlab-stack.bash ${DN_DEV_WORKSPACE}
RUN chmod +x ${DN_DEV_WORKSPACE}/entrypoint.norlab-stack.bash

FROM build-norlab-perception-ros-stack AS test

WORKDIR ${DN_DEV_WORKSPACE}

RUN echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash" \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && echo "sourcing ${DN_DEV_WORKSPACE}/install/setup.bash" \
    && source ${DN_DEV_WORKSPACE}/install/setup.bash \
    && ros2 bag play ${DN_DEV_WORKSPACE}/demo/demo

#    && ros2 launch mapper_config_template mapper.launch \

FROM build-norlab-perception-ros-stack AS final

# (NICE TO HAVE) ToDo: implement entrypoint with demo example runing

WORKDIR ${DN_DEV_WORKSPACE}

ENTRYPOINT [ "./entrypoint.norlab-stack.bash" ]
CMD [ "bash" ]

