
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS libpointmatcher-install

ARG LIBPOINTMATCHER_VERSION=latest
ENV LIBPOINTMATCHER_VERSION=${LIBPOINTMATCHER_VERSION}

ARG LIBPOINTMATCHER_CMAKE_BUILD_TYPE=Release
ARG LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG="--build-system-CI-install --compile-test"
# Note: Those env variable are used in the entrypoint build version
ENV LIBPOINTMATCHER_CMAKE_BUILD_TYPE=${LIBPOINTMATCHER_CMAKE_BUILD_TYPE}
ENV LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG=${LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG}

LABEL libpointmatcher.version="${LIBPOINTMATCHER_VERSION}"

ARG PERCEP3D_LIB_PATH=/opt/percep3d_libraries
ENV PERCEP3D_LIB_PATH=${PERCEP3D_LIB_PATH}

# ====Begin=======================================================================================
WORKDIR ${PERCEP3D_LIB_PATH}

RUN git clone --recurse-submodules https://github.com/norlab-ulaval/libpointmatcher.git

# ....Install libnabo..............................................................................
WORKDIR "${PERCEP3D_LIB_PATH}/libpointmatcher"
RUN bash libpointmatcher_dependencies_installer.bash

# ....Check Libpointmatcher dependencies installed versions........................................
RUN utest/listVersionsUbuntu.sh

# ....Install libpointmatcher.....................................................................
RUN bash libpointmatcher_installer.bash \
            --repository-version ${LIBPOINTMATCHER_VERSION} \
            --cmake-build-type ${LIBPOINTMATCHER_CMAKE_BUILD_TYPE} \
            ${LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG}


FROM libpointmatcher-install AS test

WORKDIR ${PERCEP3D_LIB_PATH}/libpointmatcher
# (NICE TO HAVE) ToDo: implement a base test casse that validate LPM is working
RUN echo "skip test"

FROM libpointmatcher-install AS final

WORKDIR ${PERCEP3D_LIB_PATH}
CMD [ "bash" ]

