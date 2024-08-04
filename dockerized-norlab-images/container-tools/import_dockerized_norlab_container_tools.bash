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
# =================================================================================================
pushd "$(pwd)" >/dev/null || exit 1

function dn::source_lib() {
  local _PATH_TO_SCRIPT

  # Note: can handle both sourcing cases
  #   i.e. from within a script or from an interactive terminal session
  _PATH_TO_SCRIPT="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  DN_PATH=$(realpath "$(dirname "${_PATH_TO_SCRIPT}")/../..")

  # ....Source DN dependencies.....................................................................
  N2ST_PATH=${DN_PATH}/utilities/norlab-shell-script-tools

  cd "${N2ST_PATH}" || exit 1
  source "import_norlab_shell_script_tools_lib.bash"

  #  # ....Source DN functions.......................................................................
  #  cd "${NBS_PATH}/src/function_library" || exit 1
  #  for each_file in "$(pwd)"/*.bash ; do
  #      source "${each_file}"
  #  done

  # ....Teardown...................................................................................
  export DN_PATH
  export N2ST_PATH

  # Set reference that the DN tools where imported with this script
  export DN_IMPORTED=true

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
  dn::source_lib
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
