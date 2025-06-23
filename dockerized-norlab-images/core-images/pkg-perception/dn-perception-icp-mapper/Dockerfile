
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS icp-mapper-install

# (CRITICAL) ToDo: on task NMO-584 end >> delete next bloc ↓↓
#ARG DEBIAN_FRONTEND=noninteractive

ARG PERCEP3D_LIB_PATH=/opt/percep3d_libraries
ENV PERCEP3D_LIB_PATH=${PERCEP3D_LIB_PATH}

# ====Begin=======================================================================================
# Reference: https://github.com/norlab-ulaval/Norlab_wiki/wiki/Tutorials:-3D-Mapping-(ros2)

##ENV CMAKE_INCLUDE_PATH=${PERCEP3D_LIB_PATH}/include
#ENV CMAKE_PREFIX_PATH="${PERCEP3D_LIB_PATH}:${PERCEP3D_LIB_PATH}/share"
#ENV CMAKE_INSTALL_PREFIX=${PERCEP3D_LIB_PATH}
##ENV libpointmatcher_INCLUDE_DIRS=${PERCEP3D_LIB_PATH}/libpointmatcher
#ENV libpointmatcher_DIR="${PERCEP3D_LIB_PATH}/share/libpointmatcher/cmake"
## ★★★ Note: Consider something around the line of libpointmatcher_DIR=${PERCEP3D_LIB_PATH}/libpointmatcher/build/install/share/libpointmatcher/cmake

# ....Install NorLab-ICP-mapper....................................................................
WORKDIR ${PERCEP3D_LIB_PATH:?'Error: env variable empty'}
RUN git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
    && mkdir -p norlab_icp_mapper/build && cd norlab_icp_mapper/build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j $(nproc) \
    && make install

FROM icp-mapper-install AS test

WORKDIR ${PERCEP3D_LIB_PATH}/norlab_icp_mapper
# (NICE TO HAVE) ToDo: implement a base test casse that validate LPM is working
RUN echo "skip test"


FROM icp-mapper-install AS final

