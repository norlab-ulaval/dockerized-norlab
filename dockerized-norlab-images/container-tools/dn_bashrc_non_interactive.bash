#!/bin/bash
# =================================================================================================
# dn_bashrc_non_interactive.bash
#
# This file act as a DN build stages temporary buffer to pass instruction that will be executed
# in non-interactive shell via the BASH_ENV environment variable.
#
# Unset BASH_ENV to disable its execution e.g.: ENV BASH_ENV =""
#
# Note:
#   "When bash is started non-interactively, to run a shell script, for example, it looks for
#    the variable BASH_ENV in the environment, expands its value if it appears there, and uses the
#    expanded value as the name of a file to read and execute. Bash behaves as if the following
#    command were executed:
#         if [ -n "$BASH_ENV" ]; then . "$BASH_ENV"; fi
#    but the value of the PATH variable is not used to search for the file name."
#   Source: See section 'Invocation' in $ man bash
#
# =================================================================================================

# ....Load Dockerized-NorLab container-tools libraries.............................................
pushd "$(pwd)" >/dev/null || exit 1
cd /dockerized-norlab/dockerized-norlab-images/container-tools || exit 1
source import_dockerized_norlab_container_tools.bash
popd >/dev/null || exit 1

# ====Build-time appended instructions=============================================================
