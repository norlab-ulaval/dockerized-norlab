#!/bin/bash
# =================================================================================================
# dn_bashrc.bash
#
# This file act as a DN build stages temporary buffer to pass instruction that will be appended
# to .bashrc at the DN project-core build stage
#
# =================================================================================================

# Skip if explicitly disabled
if [[ "${DN_DISABLE_AUTO_LOAD:-}" == "true" ]]; then
   exit 0
fi

# Only load if not already loaded (prevent double-loading)
if [[ -n "${DN_CONTAINER_TOOLS_LOADED:-}" ]]; then
    exit 0
fi

# ....Load Dockerized-NorLab container-tools libraries.............................................
if pushd "$(pwd)" >/dev/null 2>&1; then
  if cd /dockerized-norlab/dockerized-norlab-images/container-tools 2>/dev/null; then
    source import_dockerized_norlab_container_tools.bash
  fi
  popd >/dev/null 2>&1 || true
fi

# ====Build-time appended instructions=============================================================
