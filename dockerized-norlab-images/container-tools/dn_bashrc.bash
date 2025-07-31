#!/bin/bash
# =================================================================================================
# dn_bashrc.bash
#
# This file act as a DN build stages temporary buffer to pass instruction that will be appended
# to .bashrc at the DN project-core build stage
#
# =================================================================================================

# ....Load Dockerized-NorLab container-tools libraries.............................................

# Only load if not already loaded (prevent double-loading)
if [[ -z "${DN_CONTAINER_TOOLS_LOADED:-}" ]]; then
  pushd "$(pwd)" >/dev/null || exit 1
  cd /dockerized-norlab/dockerized-norlab-images/container-tools || exit 1
  source import_dockerized_norlab_container_tools.bash
  popd >/dev/null || exit 1
fi

# ====Build-time appended instructions=============================================================
