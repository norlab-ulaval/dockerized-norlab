#!/bin/bash
#
# Execute build matrix on docker compose docker-compose.libpointmatcher.yaml
#
# Usage:
#   $ bash dn_execute_compose_over_build_matrix.bash [<optional flag>] [-- <any docker cmd+arg>]
#
#   $ bash dn_execute_compose_over_build_matrix.bash -- up --build --force-recreate
#
# Arguments:
#   see function print_help_in_terminal or execute the script with the --help flag
#
# Note:
#   Dont use "set -e" in this script as it will affect the build system policy, use the --fail-fast flag instead
#

## Debug flags
#set -v
#set -x

# ....Default......................................................................................................
DOCKER_COMPOSE_CMD_ARGS='up --build --force-recreate'
BUILD_STATUS_PASS=0


# ....Pre-condition................................................................................................
if [[ ! -f  ".env.dockerized-norlab" ]]; then
  echo -e "\n[\033[1;31mERROR\033[0m] 'dn_execute_compose_over_build_matrix.bash' script must be sourced from the project root!\n Curent working directory is '$(pwd)'"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ....Load environment variables from file.........................................................................
set -o allexport
source .env.dockerized-norlab
source .env.build_matrix
set +o allexport

set -o allexport
source ./utilities/norlab-shell-script-tools/.env.project
set +o allexport

# ....Helper function..............................................................................................
## import shell functions from norlab-shell-script-tools utilities library

TMP_CWD=$(pwd)
cd ./utilities/norlab-shell-script-tools/src/function_library
source ./prompt_utilities.bash
source ./docker_utilities.bash
source ./general_utilities.bash
source ./teamcity_utilities.bash
source ./terminal_splash.bash
cd "$TMP_CWD"



function print_help_in_terminal() {
  echo -e "\n
\$ ${0} [<optional flag>] [-- <any docker cmd+arg>]
  \033[1m
    <optional argument>:\033[0m
      -h, --help          Get help
      --dockerized-norlab-version-build-matrix-override head
                          The libpointmatcher release tag. Override must be a single value
                          (default to array sequence specified in .env.build_matrix)
      --os-name-build-matrix-override ubuntu
                          The operating system name. Override must be a single value
                          (default to array sequence specified in .env.build_matrix)
      --ubuntu-version-build-matrix-override jammy
      --osx-version-build-matrix-override ventura
                          Named operating system version. Override must be a single value
                          (default to array sequence specified in .env.build_matrix)
      --docker-debug-logs
                          Set Docker builder log output for debug (i.e.BUILDKIT_PROGRESS=plain)
      --fail-fast         Exit script at first encountered error
      --ci-sitrep-run     Override LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE and
                            LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS with there respective _SITREP version

  \033[1m
    [-- <any docker cmd+arg>]\033[0m                 Any argument passed after '--' will be passed to docker compose as docker
                                              command and arguments (default to '${DOCKER_COMPOSE_CMD_ARGS}')
"
}

# ====Begin========================================================================================================
norlab_splash "${DN_SPLASH_NAME}" "${PROJECT_GIT_REMOTE_URL}"

print_formated_script_header 'dn_execute_compose_over_build_matrix.bash' "${LPM_LINE_CHAR_BUILDER_LVL1}"

# ....Script command line flags....................................................................................
while [ $# -gt 0 ]; do

  case $1 in
  --dockerized-norlab-version-build-matrix-override)
    unset LPM_MATRIX_LIBPOINTMATCHER_VERSIONS
    LPM_MATRIX_LIBPOINTMATCHER_VERSIONS=("$2")
    shift # Remove argument (--dockerized-norlab-version-build-matrix-override)
    shift # Remove argument value
    ;;
  --os-name-build-matrix-override)
    unset LPM_MATRIX_SUPPORTED_OS
    LPM_MATRIX_SUPPORTED_OS=("$2")
    shift # Remove argument (--os-name-build-matrix-override)
    shift # Remove argument value
    ;;
  --ubuntu-version-build-matrix-override)
    unset LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS
    LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS=("$2")
    shift # Remove argument (--ubuntu-version-build-matrix-override)
    shift # Remove argument value
    ;;
  --osx-version-build-matrix-override)
    unset LPM_MATRIX_OSX_SUPPORTED_VERSIONS
    LPM_MATRIX_OSX_SUPPORTED_VERSIONS=("$2")
    shift # Remove argument (--osx-version-build-matrix-override)
    shift # Remove argument value
    ;;
  --docker-debug-logs)
#    set -v
#    set -x
    export BUILDKIT_PROGRESS=plain
    shift # Remove argument (--docker-debug-logs)
    ;;
  --fail-fast)
    set -e
    shift # Remove argument (--fail-fast)
    ;;
  --ci-sitrep-run)
    shift # Remove argument (--ci-sitrep-run)
    unset LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE
    unset LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS
    LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE=("${LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE_SITREP[@]}")
    LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS=("${LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS_SITREP[@]}")
    print_msg "${MSG_DIMMED_FORMAT}ci-sitrep${MSG_END_FORMAT} run environment variable override:
        - ${MSG_DIMMED_FORMAT}LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE=(${LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE_SITREP[*]})${MSG_END_FORMAT}
        - ${MSG_DIMMED_FORMAT}LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS=(${LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS_SITREP[*]})${MSG_END_FORMAT}"
    ;;
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  --) # no more option
    shift
    DOCKER_COMPOSE_CMD_ARGS="$*"
    break
    ;;
  *) # Default case
    break
    ;;
  esac

done


# ..................................................................................................................
print_msg "Build images specified in ${MSG_DIMMED_FORMAT}'docker-compose.libpointmatcher.yaml'${MSG_END_FORMAT} following ${MSG_DIMMED_FORMAT}.env.build_matrix${MSG_END_FORMAT}"

## Freeze build matrix env variable to prevent override by dn_execute_compose.bash when reloading .env/build_matrix
FREEZED_LPM_MATRIX_LIBPOINTMATCHER_VERSIONS=("${LPM_MATRIX_LIBPOINTMATCHER_VERSIONS[@]}")
FREEZED_LPM_MATRIX_SUPPORTED_OS=("${LPM_MATRIX_SUPPORTED_OS[@]}")
FREEZED_LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS=("${LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS[@]}")
FREEZED_LPM_MATRIX_OSX_SUPPORTED_VERSIONS=("${LPM_MATRIX_OSX_SUPPORTED_VERSIONS[@]}")
FREEZED_LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE=("${LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE[@]}")


print_msg "Environment variables ${MSG_EMPH_FORMAT}(build matrix)${MSG_END_FORMAT} set for compose:\n
${MSG_DIMMED_FORMAT}    LPM_MATRIX_LIBPOINTMATCHER_VERSIONS=(${FREEZED_LPM_MATRIX_LIBPOINTMATCHER_VERSIONS[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE=(${FREEZED_LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    LPM_MATRIX_SUPPORTED_OS=(${FREEZED_LPM_MATRIX_SUPPORTED_OS[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS=(${FREEZED_LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    LPM_MATRIX_OSX_SUPPORTED_VERSIONS=(${FREEZED_LPM_MATRIX_OSX_SUPPORTED_VERSIONS[*]}) ${MSG_END_FORMAT}
"

# Note: EACH_LPM_VERSION is used for container labeling and to fetch the repo at release tag
for EACH_LPM_VERSION in "${FREEZED_LPM_MATRIX_LIBPOINTMATCHER_VERSIONS[@]}"; do
  if [[ ${TEAMCITY_VERSION} ]]; then
    echo -e "##teamcity[blockOpened name='${MSG_BASE_TEAMCITY} ${EACH_LPM_VERSION}']"
  fi

  for EACH_OS_NAME in "${FREEZED_LPM_MATRIX_SUPPORTED_OS[@]}"; do
    unset CRAWL_OS_VERSIONS

    if [[ ${EACH_OS_NAME} == 'ubuntu' ]]; then
      CRAWL_OS_VERSIONS=("${FREEZED_LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS[@]}")
    elif [[ ${EACH_OS_NAME} == 'osx' ]]; then
      CRAWL_OS_VERSIONS=("${FREEZED_LPM_MATRIX_OSX_SUPPORTED_VERSIONS[@]}")
    else
      print_msg_error_and_exit "${EACH_OS_NAME} not supported!"
    fi

    if [[ -z ${CRAWL_OS_VERSIONS} ]]; then
        print_msg_error_and_exit "Can't crawl ${EACH_OS_NAME} supported version array because it's empty!"
    fi

    if [[ ${TEAMCITY_VERSION} ]]; then
      echo -e "##teamcity[blockOpened name='${MSG_BASE_TEAMCITY} ${EACH_OS_NAME}']"
    fi

    for EACH_OS_VERSION in "${CRAWL_OS_VERSIONS[@]}"; do
#      export LPM_JOB_ID=${LPM_JOB_ID}

      if [[ ${TEAMCITY_VERSION} ]]; then
        echo -e "##teamcity[blockOpened name='${MSG_BASE_TEAMCITY} ${EACH_OS_VERSION}']"
      fi

      for EACH_CMAKE_BUILD_TYPE in "${FREEZED_LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE[@]}"; do

        # shellcheck disable=SC2034
        SHOW_SPLASH_EC='false'

        if [[ ${TEAMCITY_VERSION} ]]; then
          echo -e "##teamcity[blockOpened name='${MSG_BASE_TEAMCITY} execute lpm_execute_compose.bash' description='${MSG_DIMMED_FORMAT_TEAMCITY} --dockerized-norlab-version ${EACH_LPM_VERSION} --libpointmatcher-cmake-build-type ${EACH_CMAKE_BUILD_TYPE} --os-name ${EACH_OS_NAME} --os-version ${EACH_OS_VERSION} -- ${DOCKER_COMPOSE_CMD_ARGS}${MSG_END_FORMAT_TEAMCITY}|n']"
          echo " "
        fi

        source dn_execute_compose.bash --dockerized-norlab-version "${EACH_LPM_VERSION}" \
                                          --libpointmatcher-cmake-build-type "${EACH_CMAKE_BUILD_TYPE}" \
                                          --os-name "${EACH_OS_NAME}" \
                                          --os-version "${EACH_OS_VERSION}" \
                                          -- "${DOCKER_COMPOSE_CMD_ARGS}"

        # ....Collect image tags exported by dn_execute_compose.bash..............................................
        # Global: Read 'DOCKER_EXIT_CODE' env variable exported by function show_and_execute_docker
        if [[ ${DOCKER_EXIT_CODE} == 0 ]]; then
          MSG_STATUS="${MSG_DONE_FORMAT}Pass ${MSG_DIMMED_FORMAT}›"
          MSG_STATUS_TC_TAG="Pass ›"
        else
          MSG_STATUS="${MSG_ERROR_FORMAT}Fail ${MSG_DIMMED_FORMAT}›"
          MSG_STATUS_TC_TAG="Fail ›"
          BUILD_STATUS_PASS=$DOCKER_EXIT_CODE

          if [[ ${TEAMCITY_VERSION} ]]; then
            # Fail the build › Will appear on the TeamCity Build Results page
            echo -e "##teamcity[buildProblem description='BUILD FAIL with docker exit code: ${BUILD_STATUS_PASS}']"
          fi
        fi

        # Collect image tags exported by dn_execute_compose.bash
        # Global: Read 'DN_IMAGE_TAG' env variable exported by dn_execute_compose.bash
        if [[ ${EACH_CMAKE_BUILD_TYPE} == 'None' ]] || [[ -z ${EACH_CMAKE_BUILD_TYPE} ]]; then
          IMAGE_TAG_CRAWLED=("${IMAGE_TAG_CRAWLED[@]}" "${MSG_STATUS} ${DN_IMAGE_TAG}")
          IMAGE_TAG_CRAWLED_TC=("${IMAGE_TAG_CRAWLED_TC[@]}" "${MSG_STATUS_TC_TAG} ${DN_IMAGE_TAG}")
        else
          IMAGE_TAG_CRAWLED=("${IMAGE_TAG_CRAWLED[@]}" "${MSG_STATUS} ${DN_IMAGE_TAG} Compile mode: ${EACH_CMAKE_BUILD_TYPE}")
          IMAGE_TAG_CRAWLED_TC=("${IMAGE_TAG_CRAWLED_TC[@]}" "${MSG_STATUS_TC_TAG} ${DN_IMAGE_TAG} Compile mode: ${EACH_CMAKE_BUILD_TYPE}")
        fi
        # .........................................................................................................

        if [[ ${TEAMCITY_VERSION} ]]; then
          echo -e "##teamcity[blockClosed name='${MSG_BASE_TEAMCITY} execute lpm_execute_compose.bash']"
        fi

      done

      if [[ ${TEAMCITY_VERSION} ]]; then
        echo -e "##teamcity[blockClosed name='${MSG_BASE_TEAMCITY} ${EACH_OS_VERSION}']"
      fi

    done

    if [[ ${TEAMCITY_VERSION} ]]; then
      echo -e "##teamcity[blockClosed name='${MSG_BASE_TEAMCITY} ${EACH_OS_NAME}']"
    fi

  done
  if [[ ${TEAMCITY_VERSION} ]]; then
    echo -e "##teamcity[blockClosed name='${MSG_BASE_TEAMCITY} ${EACH_LPM_VERSION}']"
  fi
done

echo " "
print_msg "Environment variables ${MSG_EMPH_FORMAT}(build matrix)${MSG_END_FORMAT} used by compose:\n
${MSG_DIMMED_FORMAT}    LPM_MATRIX_LIBPOINTMATCHER_VERSIONS=(${FREEZED_LPM_MATRIX_LIBPOINTMATCHER_VERSIONS[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE=(${FREEZED_LPM_MATRIX_LIBPOINTMATCHER_CMAKE_BUILD_TYPE[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    LPM_MATRIX_SUPPORTED_OS=(${FREEZED_LPM_MATRIX_SUPPORTED_OS[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS=(${FREEZED_LPM_MATRIX_UBUNTU_SUPPORTED_VERSIONS[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    LPM_MATRIX_OSX_SUPPORTED_VERSIONS=(${FREEZED_LPM_MATRIX_OSX_SUPPORTED_VERSIONS[*]}) ${MSG_END_FORMAT}
"

print_msg_done "FINAL › Build matrix completed with command

${MSG_DIMMED_FORMAT}    $ docker compose -f docker-compose.libpointmatcher.yaml ${DOCKER_COMPOSE_CMD_ARGS} ${MSG_END_FORMAT}

Status of tag crawled:
"
for tag in "${IMAGE_TAG_CRAWLED[@]}" ; do
    echo -e "   ${tag}${MSG_END_FORMAT}"
done

print_formated_script_footer 'dn_execute_compose_over_build_matrix.bash' "${LPM_LINE_CHAR_BUILDER_LVL1}"

# ====TeamCity service message=====================================================================================
if [[ ${TEAMCITY_VERSION} ]]; then
  # Tag added to the TeamCity build via a service message
  for tc_build_tag in "${IMAGE_TAG_CRAWLED_TC[@]}" ; do
      echo -e "##teamcity[addBuildTag '${tc_build_tag}']"
  done
fi
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"
exit $BUILD_STATUS_PASS
