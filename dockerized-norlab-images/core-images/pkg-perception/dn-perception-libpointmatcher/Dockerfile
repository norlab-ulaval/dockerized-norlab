
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS libpointmatcher-install

ARG LIBPOINTMATCHER_VERSION=head
ENV LIBPOINTMATCHER_VERSION=${LIBPOINTMATCHER_VERSION}

ARG LIBPOINTMATCHER_CMAKE_BUILD_TYPE=Release
ARG LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG="--compile-test"
# Note: Those env variable are used in the entrypoint build version
ENV LIBPOINTMATCHER_CMAKE_BUILD_TYPE=${LIBPOINTMATCHER_CMAKE_BUILD_TYPE}
ENV LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG=${LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG}

LABEL libpointmatcher.version="${LIBPOINTMATCHER_VERSION}"

ARG PERCEP3D_LIB_PATH=/opt/percep3d_libraries
ENV PERCEP3D_LIB_PATH=${PERCEP3D_LIB_PATH}

# ====Begin=======================================================================================
# ....Install libpointmatcher.....................................................................
WORKDIR ${PERCEP3D_LIB_PATH}

## (NICE TO HAVE) ToDo: NMO-351 ♻︎ → add install path feature to lpm_install_dependencies_ubuntu.bash
## ToDo: NMO-352 ♻︎ → Update libpointmatcher github domain on repo transfer from ethz-asl to norlab-ulaval
#RUN git clone https://github.com/ethz-asl/libpointmatcher.git \
#    && cd ./libpointmatcher/build_system \
#    && source ./ubuntu/lpm_install_python_dev_tools.bash \
#    && cd ./ubuntu/ \
#    && source ./lpm_install_dependencies_ubuntu.bash
#
#WORKDIR ${PERCEP3D_LIB_PATH}/libpointmatcher/build_system/ubuntu/
#RUN source lpm_install_libpointmatcher_ubuntu.bash \
#            --build-system-CI-install \
#            --libpointmatcher-version ${LIBPOINTMATCHER_VERSION} \
#            --cmake-build-type ${LIBPOINTMATCHER_CMAKE_BUILD_TYPE} \
#            ${LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG}

# ////Alt install//////////////////////////////////////////////////////////////////////////////////
#ENV CMAKE_PREFIX_PATH="${PERCEP3D_LIB_PATH}"
#ENV DN_CMAKE_INSTALL_PREFIX=/usr/local/install
ENV DN_CMAKE_INSTALL_PREFIX=/usr/local
#CMAKE_PREFIX_PATH:PATH=/usr/local
#-D CMAKE_INSTALL_PREFIX=/usr/local

RUN apt-get update \
    && apt-get install --assume-yes \
        libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install --assume-yes \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${PERCEP3D_LIB_PATH}

RUN git clone https://github.com/ethz-asl/libnabo.git \
    && cd libnabo \
    && mkdir build && cd build \
    && cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo \
          -D CMAKE_INSTALL_PREFIX=${DN_CMAKE_INSTALL_PREFIX} \
#          -D CMAKE_INSTALL_PREFIX=${PERCEP3D_LIB_PATH}/libnabo/build/install \
          "${PERCEP3D_LIB_PATH}/libnabo" \
    && make -j $(nproc) \
    && make install
#    && make test \


RUN apt-get update \
    && apt-get install --assume-yes \
      libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/ethz-asl/libpointmatcher.git \
    && cd libpointmatcher/ \
    && mkdir build/ && cd build/ \
    && cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo \
             -D CMAKE_INSTALL_PREFIX=${DN_CMAKE_INSTALL_PREFIX} \
#             -D CMAKE_INSTALL_PREFIX=${PERCEP3D_LIB_PATH}/libpointmatcher/build/install \
#             -D BUILD_TESTS=TRUE \
             -D LIBNABO_INSTALL_DIR="${DN_CMAKE_INSTALL_PREFIX}/libnabo" \
             "${PERCEP3D_LIB_PATH}/libpointmatcher" \
    && make -j $(nproc) \
    && make install

#ENV libpointmatcher_DIR=${PERCEP3D_LIB_PATH}/libpointmatcher/build/install/share/libpointmatcher/cmake
ENV libpointmatcher_DIR=${DN_CMAKE_INSTALL_PREFIX}/share/libpointmatcher/cmake


FROM libpointmatcher-install AS test

WORKDIR ${PERCEP3D_LIB_PATH}/libpointmatcher/build_system/ubuntu
RUN source lpm_execute_libpointmatcher_unittest.bash

FROM libpointmatcher-install AS final

WORKDIR ${PERCEP3D_LIB_PATH}
CMD [ "bash" ]

