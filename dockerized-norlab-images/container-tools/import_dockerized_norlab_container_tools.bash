#!/bin/bash
# =================================================================================================
# Import Dockerized-NorLab function library and dependencies
#
# Usage in a interactive terminal session:
#
#   $ cd <path/to>/dockerized-norlab/dockerized-norlab-images/container-tools/
#   $ PROJECT_PROMPT_NAME=MySuperProject
#   $ source import_dockerized_norlab_container_tools.bash
#
# Usage from within a shell script:
#
#   #!/bin/bash
#   cd <my/superproject/root>/dockerized-norlab/dockerized-norlab-images/container-tools/
#   source import_dockerized_norlab_container_tools.bash
#
# Globals:
#   write DN_PATH
#   write N2ST_PATH
#   write DN_CONTAINER_TOOLS_LOADED
#   write DN_SHOW_DEBUG_INFO
#
# =================================================================================================

#DN_SHOW_DEBUG_INFO=false # (CRITICAL) ToDo: on task end >> mute this line ←

function dn::source_lib() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)
  local script_path
  local target_path

  if [[ -n "$( declare -f n2st::print_msg )" ]] && [[ "${DN_CONTAINER_TOOLS_LOADED:-}" == true ]]; then
    if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
      echo -e "\033[1;33m[DN trace]\033[0m Dockerized-NorLab container tools are already loaded. Skip import."
    fi
    return 0
  else
    if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
      echo -e "\033[1;33m[DN trace]\033[0m Import Dockerized-NorLab container tools..."
    fi
  fi


  # ....Find path to script........................................................................
  if [[ -z ${DN_PATH} ]]; then
    # Note: can handle both sourcing cases
    #   i.e. from within a script or from an interactive terminal session
    # Check if running interactively
    if [[ $- == *i* ]] || [[ -n "$PS1" ]]; then
      # Case: running in an interactive session
      target_path=$(realpath .)
    else
      # Case: running in an non-interactive session
      script_path="$(realpath -q "${BASH_SOURCE[0]:-.}")"
      target_path="$(dirname "${script_path}")"
    fi

    if [[ "${DN_SHOW_DEBUG_INFO}" == true ]] && [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
      echo -e "\033[1;33m[DN trace]\033[0m Execute import_dockerized_norlab_container_tools.bash from ${BASH_SOURCE[2]}
      ${MSG_DIMMED_FORMAT}
      BASH_SOURCE: ${BASH_SOURCE[*]}
      tmp_cwd: ${tmp_cwd}
      script_path: ${script_path}
      target_path: ${target_path}
      \$0: $0
      ${MSG_END_FORMAT}"
    fi

    if [[ "$( basename "${target_path}" )" != "container-tools" ]]; then
      echo -e "\n[\033[1;31mDN error\033[0m] Can't find directory 'container-tools' at ${target_path}!" 1>&2 && return 1
    fi

    DN_PATH=$(realpath "${target_path}/../..")
  fi

  N2ST_PATH=${DN_PATH}/utilities/norlab-shell-script-tools

  export DN_PATH
  export N2ST_PATH

  # ....Source DN dependencies.....................................................................
  cd "${N2ST_PATH}" || return 1
  source "import_norlab_shell_script_tools_lib.bash" || return 1

  # ....Source DN container-tools..................................................................
  cd "${DN_PATH}/dockerized-norlab-images/container-tools" || return 1
  source dn_source_ros2.bash

  # Note: Do not `export -f [n2st|dn]::<function-name>` in the container as it polute the
  #       container environment variables space and make it hard to consult with 'printenv' and
  #       'env' for example both at runtime and in CI ⚠️.

  # ....Teardown...................................................................................
  # Set reference that the DN tools where imported with this script
  export DN_CONTAINER_TOOLS_LOADED=true

  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  MSG_ERROR_FORMAT="\033[1;31m"
  MSG_END_FORMAT="\033[0m"
  echo -e "${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dn::source_lib || exit 1
fi
