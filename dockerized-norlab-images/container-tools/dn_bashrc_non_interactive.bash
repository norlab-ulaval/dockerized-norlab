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

_DN_DEBUG_LOG=true

# Skip if explicitly disabled
if [[ "${DN_DISABLE_AUTO_LOAD:-}" == "true" ]]; then
    if [[ ${_DN_DEBUG_LOG} == true ]]; then
      echo "DN_DISABLE_AUTO_LOAD: ${DN_DISABLE_AUTO_LOAD}. Skip dn_bashrc_non_interactive.bash"
    fi
   exit 0
fi

# Only load if not already loaded (prevent double-loading)
if [[ -n "${DN_CONTAINER_TOOLS_LOADED:-}" ]]; then
    if [[ ${_DN_DEBUG_LOG} == true ]]; then
      echo "DN_CONTAINER_TOOLS_LOADED: ${DN_CONTAINER_TOOLS_LOADED} -> already loaded. Skip dn_bashrc_non_interactive.bash"
    fi
    exit 0
fi


# ....Smart detection to avoid loading in nested bash calls.......................................
# Strategy: Only load for "top-level" bash processes, not nested ones spawned by tools

# Method 1: Check if we're in a direct RUN instruction vs nested call
# Docker RUN instructions typically have SHLVL=1, while nested calls have higher values
if [[ "${SHLVL:-1}" -gt 2 ]]; then
    if [[ ${_DN_DEBUG_LOG} == true ]]; then
      echo "SHLVL: ${SHLVL} > 1 -> assume executed by a dockerfile RUN instruction. Skip dn_bashrc_non_interactive.bash"
    fi
    exit 0
fi

# Method 2: Check process ancestry for known problematic commands
# Get the command line of parent processes
if command -v ps >/dev/null 2>&1; then
    # Check if any parent process is a known problematic command
    parent_processes=$(ps -o comm= -p $PPID 2>/dev/null || true)
    for process in $parent_processes; do
        case "$process" in
            colcon|cmake|rosdep|apt-get|dpkg|debconf|aptitude|pip|pip3|python*|make)
                if [[ ${_DN_DEBUG_LOG} == true ]]; then
                  echo "process: ${process} in colcon|cmake|rosdep|apt-get|dpkg|debconf|aptitude|pip|pip3|python*|make -> assume executed by a problematic command. Skip dn_bashrc_non_interactive.bash"
                fi
                exit 0
                ;;
        esac
    done
fi

# Method 3: Check for specific environment markers that indicate we're in a tool
# Many build tools set specific environment variables
if [[ -n "${COLCON_LOG_PATH:-}" ]] ||
   [[ -n "${CMAKE_CURRENT_BINARY_DIR:-}" ]] ||
   [[ -n "${_ROSDEP_RUNNING:-}" ]] ||
   [[ -n "${PIP_RUNNING:-}" ]]; then
     if [[ ${_DN_DEBUG_LOG} == true ]]; then
       echo "Specific environment markers is set
  COLCON_LOG_PATH: ${COLCON_LOG_PATH}
  CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}
  _ROSDEP_RUNNING: ${_ROSDEP_RUNNING}
  PIP_RUNNING: ${PIP_RUNNING}
 -> assume executed by a problematic command. Skip dn_bashrc_non_interactive.bash"
     fi
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
