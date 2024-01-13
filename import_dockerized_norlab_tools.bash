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
# =================================================================================================

# ToDo: unit-test (ref task NMO-486 test: implement import_dockerized_norlab_tools.bash bats tests)

MSG_DIMMED_FORMAT="\033[1;2m"
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

function dn::source_lib(){
  local TMP_CWD
  TMP_CWD=$(pwd)

  # Note: can handle both sourcing cases
  #   i.e. from within a script or from an interactive terminal session
  _PATH_TO_SCRIPT="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  _REPO_ROOT="$(dirname "${_PATH_TO_SCRIPT}")"

  # ....Load environment variables from file.......................................................
  cd "${_REPO_ROOT}" || exit 1
  set -o allexport
  source .env.dockerized-norlab-build-system
  source .env.dockerized-norlab-project
  set +o allexport

  # ....Source DN dependencies.....................................................................
  cd "${N2ST_PATH:?"Variable not set"}" || exit 1
  source "import_norlab_shell_script_tools_lib.bash"

  cd "${NBS_PATH:?"Variable not set"}" || exit 1
  source "import_norlab_build_system_lib.bash"

#  # ....Source DN functions.......................................................................
#  cd "${NBS_PATH}/src/function_library" || exit 1
#  for each_file in "$(pwd)"/*.bash ; do
#      source "${each_file}"
#  done

  # Set reference that the DN tools where imported with this script
  export DN_IMPORTED=true

  # ....Teardown...................................................................................
  cd "${TMP_CWD}"
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} This script must be sourced i.e.: $ source $( basename "$0" )" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dn::source_lib
fi

