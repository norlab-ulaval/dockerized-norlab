#!/bin/bash
# ===================================================================================================
# This a TeamCity (TC) build step script with configuration parameter support for custom run.
#
# The script reads, initializes, and exports various environment variables from TC configuration
# parameters set via the custom run UI. It also checks conditions and adds specific flags to
# DN_BUILD_ALL_FLAGS. Finally, it initiates a build process by sourcing the dn_build_all.bash
# script.
#
# Globals:
# Read/Write: NBS_OVERRIDE_BUILD_MATRIX_MAIN, NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG,
#             NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY, DN_BUILD_ALL_FLAGS, DN_PUSH_HOT_TAG
#
# Outputs:
# Writes the environment variables, added flags to stdout.
#
# Usage:
#    copy the code into the "custom script" cell of a TC build step
#
# ===================================================================================================

# ....NBS_OVERRIDE build matrix main............................
NBS_OVERRIDE_BUILD_MATRIX_MAIN=%NBS_OVERRIDE_BUILD_MATRIX_MAIN%
echo "NBS_OVERRIDE_BUILD_MATRIX_MAIN=%NBS_OVERRIDE_BUILD_MATRIX_MAIN%"
if [[ -n "$NBS_OVERRIDE_BUILD_MATRIX_MAIN"  ]]; then
  export NBS_OVERRIDE_BUILD_MATRIX_MAIN
  echo "execute: export NBS_OVERRIDE_BUILD_MATRIX_MAIN"
fi

# ....NBS_OVERRIDE add docker cmd and flag......................
NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG=%NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG%
echo "NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG=%NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG%"
if [[ -n "${NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG}"  ]]; then
  export NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG
  echo "execute: export NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG"
fi

# ....NBS_OVERRIDE dotenv build matrix array....................
# Convert comma delimited list of word to an array of word
NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY=($(echo %NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY% | tr "," "\n"))
echo "NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY=("
for each in "${NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY[@]}" ; do
    echo "  ${each}"
done
echo ")"

if [[ -n "${NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY}"  ]]; then
  export NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY
  echo "execute: export NBS_OVERRIDE_DOTENV_BUILD_MATRIX_ARRAY"
fi

# ....Setup flags for dn_build_all.bash.........................
DN_BUILD_ALL_FLAGS=()
# Add flag from TeamCity custom parameters
DN_BUILD_ALL_FLAGS+=($(echo %DN_BUILD_ALL_FLAGS% | tr " " "\n"))
DN_PUSH_HOT_TAG=%DN_PUSH_HOT_TAG%
if [[ ${DN_PUSH_HOT_TAG} == true ]]; then
  DN_BUILD_ALL_FLAGS+=("--force-push" "--dockerized-norlab-version-build-matrix-override" "hot")
fi
if [[ -n "${DN_BUILD_ALL_FLAGS}" ]]; then
  echo "DN_BUILD_ALL_FLAGS=("
  for each in "${DN_BUILD_ALL_FLAGS[@]}" ; do
      echo "  ${each}"
  done
  echo ")"
fi

# ....Final.....................................................
echo -e "execute: source dockerized-norlab-scripts/build_script/dn_build_all.bash ${DN_BUILD_ALL_FLAGS[*]}"
source dockerized-norlab-scripts/build_script/dn_build_all.bash "${DN_BUILD_ALL_FLAGS[@]}"
