#!/bin/bash
# =================================================================================================
# Import Dockerized-NorLab function library and dependencies
#
# Usage in a interactive terminal session:
#
#   $ cd <path/to/dockerized-norlab/>
#   $ PROJECT_PROMPT_NAME=MySuperProject
#   $ source import_dockerized_norlab_tools.bash
#
# Usage from within a shell script:
#
#   #!/bin/bash
#   cd <my/superproject/root>/utilities/dockerized-norlab
#   source import_dockerized_norlab_tools.bash
#
# Globals:
#   write DN_IMPORTED
#
# =================================================================================================


function dn::source_lib() {
  # ....Setup......................................................................................
  local debug_log=false
  local tmp_cwd
  tmp_cwd=$(pwd)
  local script_path
  local target_path

  if [[ "${DN_IMPORTED:-}" == true ]]; then
    if [[ ${debug_log} == true ]]; then
      n2st::print_msg "Dockerized-NorLab tools are already loaded. Skip import."
    fi
    return 0
  else
    if [[ ${debug_log} == true ]]; then
      echo "Import Dockerized-NorLab tools..."
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

    if [[ ${debug_log} == true ]]; then
      echo "
      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
      BASH_SOURCE: ${BASH_SOURCE[*]}

      tmp_cwd: ${tmp_cwd}
      script_path: ${script_path}
      target_path: ${target_path}

      realpath: $(realpath .)
      \$0: $0
      <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      "
    fi
  else
    target_path="${DN_PATH}"
  fi

  cd "${target_path}" || return 1

  if [[ ! -f .env.dockerized-norlab-build-system ]]; then
    echo -e "\n[\033[1;31mDN error\033[0m] Can't find Dockerized-NorLab repository root!" 1>&2
    return 1
  fi

  # ....Load environment variables from file.......................................................
  set -o allexport
  source .env.dockerized-norlab-project || return 1
  set +o allexport

  # ....Source DN dependencies.....................................................................
  # . . Import NBS lib . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . ..
  set -o allexport
  source .env.dockerized-norlab-build-system || return 1
  set +o allexport

  cd "${NBS_PATH:?"Variable not set"}" || return 1
  source "import_norlab_build_system_lib.bash" || return 1

  # . . Import N2ST lib . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cd "${target_path}" || return 1
  set -o allexport
  source .env.dockerized-norlab-build-system || return 1
  set +o allexport

  # Note: Import last so that N2ST_PATH point to the DN submodule path instead of NBS submodule path
  cd "${N2ST_PATH:?"Variable not set"}" || return 1
  source "import_norlab_shell_script_tools_lib.bash" || return 1

  # ....Export loaded functions....................................................................
  for func in $(compgen -A function | grep -e n2st:: -e nbs::); do
    # shellcheck disable=SC2163
    export -f "${func}"
  done

  # ....Teardown...................................................................................
  # Set reference that the DN tools where imported with this script
  export DN_IMPORTED=true

  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  MSG_ERROR_FORMAT="\033[1;31m"
  MSG_END_FORMAT="\033[0m"

  # This script is being run, ie: __name__="__main__"
  echo -e "${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dn::source_lib || exit 1
fi
