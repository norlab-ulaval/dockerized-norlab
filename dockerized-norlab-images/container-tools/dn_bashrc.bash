#!/bin/bash
# =================================================================================================
# dn_bashrc.bash
#
# This file act as a DN build stages temporary buffer to pass instruction that will be appended
# to .bashrc at the DN project-core build stage
#
# =================================================================================================

# ....Load Dockerized-NorLab container-tools libraries.............................................
if pushd "$(pwd)" >/dev/null 2>&1; then
  if cd /dockerized-norlab/dockerized-norlab-images/container-tools 2>/dev/null; then
    source import_dockerized_norlab_container_tools.bash
  fi
  popd >/dev/null 2>&1 || true
fi

# ====Build-time appended instructions=============================================================
