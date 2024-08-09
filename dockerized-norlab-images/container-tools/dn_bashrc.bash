#!/bin/bash
# =================================================================================================
# dn_bashrc.bash
#
# This file act as a DN build stages temporary buffer to pass instruction that will be appended
# to .bashrc at the DN project-core build stage
#
# =================================================================================================

# ....Load Dockerized-NorLab container-tools libraries.............................................
pushd "$(pwd)" >/dev/null || exit 1
cd /dockerized-norlab/dockerized-norlab-images/container-tools || exit 1
source import_dockerized_norlab_container_tools.bash
popd >/dev/null || exit 1

# ====Build-time appended instructions=============================================================
