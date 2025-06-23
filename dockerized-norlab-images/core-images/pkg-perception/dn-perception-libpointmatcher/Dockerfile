
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS libpointmatcher-install

# (CRITICAL) ToDo: on task NMO-584 end >> delete next bloc ↓↓
#ARG DEBIAN_FRONTEND=noninteractive

ARG LIBPOINTMATCHER_VERSION=latest
ENV LIBPOINTMATCHER_VERSION=${LIBPOINTMATCHER_VERSION}

ARG LIBPOINTMATCHER_CMAKE_BUILD_TYPE=Release
ENV LIBPOINTMATCHER_CMAKE_BUILD_TYPE=${LIBPOINTMATCHER_CMAKE_BUILD_TYPE}

LABEL libpointmatcher.version="${LIBPOINTMATCHER_VERSION}"

ARG PERCEP3D_LIB_PATH=/opt/percep3d_libraries
ENV PERCEP3D_LIB_PATH=${PERCEP3D_LIB_PATH}
ARG CMAKE_INSTALL_PREFIX="-DCMAKE_INSTALL_PREFIX=${PERCEP3D_LIB_PATH:?err}"

# ====Begin=======================================================================================
WORKDIR ${PERCEP3D_LIB_PATH}

RUN git clone --recurse-submodules https://github.com/norlab-ulaval/libpointmatcher.git

WORKDIR "${PERCEP3D_LIB_PATH}/libpointmatcher"


RUN <<EOF
    # ....Install libpointmatcher dependencies.....................................................
    export APPEND_TO_CMAKE_FLAG=( "${CMAKE_INSTALL_PREFIX}" )

    # //// Temp solution //////////////////////////////////////////////////////////////////////////
    ## (Priority) ToDo: unmute when installer as the option to skip doc install (ref task NMO-605).
    ##
    #yes | bash libpointmatcher_dependencies_installer.bash || exit 1
    ##
    ## Use manual install step in the mean time
    set -o allexport && source .env.libpointmatcher && set +o allexport || exit 1
    cd "${PERCEP3D_LIB_PATH}/libpointmatcher/build_system/ubuntu" || exit 1
    yes | source lpm_install_dependencies_general_ubuntu.bash || exit 1
    cd "${PERCEP3D_LIB_PATH}/libpointmatcher/build_system/ubuntu" || exit 1
    yes | source lpm_install_dependencies_libnabo_ubuntu.bash || exit 1
    # \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Temp solution \\\\

    # ....Check Libpointmatcher dependencies installed versions....................................
    cd "${PERCEP3D_LIB_PATH}"/libpointmatcher || exit 1
    chmod +x ./utest/listVersionsUbuntu.sh
    utest/listVersionsUbuntu.sh
EOF

RUN <<EOF
    # ....Install libpointmatcher..................................................................

    # (CRITICAL) ToDo: fix(LPM installer): `--build-system-CI-install` logic should be the default
    #   in LPM installer.
    #   Ref task NMO-602 fix(installer): set LPM user installer to skip repo clone step by default

    declare -a LPM_INSTALL_FLAG=()
    LPM_INSTALL_FLAG+=(--build-system-CI-install)
    LPM_INSTALL_FLAG+=(--compile-test)

    export APPEND_TO_CMAKE_FLAG=( "${CMAKE_INSTALL_PREFIX}" )
    bash libpointmatcher_installer.bash \
      --repository-version "${LIBPOINTMATCHER_VERSION}" \
      --cmake-build-type "${LIBPOINTMATCHER_CMAKE_BUILD_TYPE}" \
      "${LPM_INSTALL_FLAG[@]}"  \
      || exit 1

EOF

FROM libpointmatcher-install AS test
WORKDIR ${PERCEP3D_LIB_PATH}/libpointmatcher/build_system/ubuntu
RUN source lpm_execute_libpointmatcher_unittest.bash

FROM libpointmatcher-install AS final

