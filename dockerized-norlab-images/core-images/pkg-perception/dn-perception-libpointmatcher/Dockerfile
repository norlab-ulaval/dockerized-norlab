
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS libpointmatcher-install

# (CRITICAL) ToDo: on task NMO-584 end >> delete next bloc ↓↓
#ARG DEBIAN_FRONTEND=noninteractive

ARG LIBPOINTMATCHER_VERSION=latest
ENV LIBPOINTMATCHER_VERSION=${LIBPOINTMATCHER_VERSION}

ARG LIBPOINTMATCHER_CMAKE_BUILD_TYPE=Release
ARG LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG="--build-system-CI-install --compile-test"

# Note: The following env variable are used in the entrypoint build version
ENV LIBPOINTMATCHER_CMAKE_BUILD_TYPE=${LIBPOINTMATCHER_CMAKE_BUILD_TYPE}
ENV LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG=${LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG}

LABEL libpointmatcher.version="${LIBPOINTMATCHER_VERSION}"

ARG PERCEP3D_LIB_PATH=/opt/percep3d_libraries
ENV PERCEP3D_LIB_PATH=${PERCEP3D_LIB_PATH}

# ====Begin=======================================================================================
WORKDIR ${PERCEP3D_LIB_PATH}

RUN <<EOF
    git clone --recurse-submodules https://github.com/norlab-ulaval/libpointmatcher.git

    # ....Install libnabo..........................................................................
    cd "${PERCEP3D_LIB_PATH}/libpointmatcher" || exit 1


    # //// Temp solution //////////////////////////////////////////////////////////////////////////
    ## (Priority) ToDo: unmute when installer as the option to skip doc install (ref task NMO-605).
    ##
    #yes | bash libpointmatcher_dependencies_installer.bash || exit 1
    ##
    ## Use manual install step in the mean time
    set -o allexport && source .env.libpointmatcher && set +o allexport || exit 1
    cd "${PERCEP3D_LIB_PATH}"/libpointmatcher/build_system/ubuntu || exit 1
    yes | source lpm_install_dependencies_general_ubuntu.bash || exit 1
    cd "${PERCEP3D_LIB_PATH}"/libpointmatcher/build_system/ubuntu || exit 1
    yes | source lpm_install_dependencies_libnabo_ubuntu.bash || exit 1
    # \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Temp solution \\\\


    # ....Check Libpointmatcher dependencies installed versions....................................
    bash utest/listVersionsUbuntu.sh

    # ....Install libpointmatcher..................................................................
    export APPEND_TO_CMAKE_FLAG=( "-D CMAKE_INSTALL_PREFIX=${PERCEP3D_LIB_PATH:?err}" )
    # (CRITICAL) ToDo: fix(LPM installer): `--build-system-CI-install` logic should be the default
    #   in LPM installer.
    #   Ref task NMO-602 fix(installer): set LPM user installer to skip repo clone step by default
    bash libpointmatcher_installer.bash \
      --repository-version ${LIBPOINTMATCHER_VERSION} \
      --cmake-build-type ${LIBPOINTMATCHER_CMAKE_BUILD_TYPE} \
      ${LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG}  \
      || exit 1

EOF

FROM libpointmatcher-install AS test
WORKDIR ${PERCEP3D_LIB_PATH}/libpointmatcher/build_system/ubuntu
run bash lpm_execute_libpointmatcher_unittest.bash

FROM libpointmatcher-install AS final

