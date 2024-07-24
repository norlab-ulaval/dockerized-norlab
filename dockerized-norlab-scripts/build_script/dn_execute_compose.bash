#!/bin/bash
# =================================================================================================
#
# Build and run a single container based on docker compose docker-compose.yaml
#
# Usage example:
#   $ cd <path/to/dockerized-norlab/root>
#   $ source dockerized-norlab-scripts/build_script/dn_execute_compose.bash
#   $ dn::execute_compose docker-compose.project.yaml -- build --push dependencies-core
#   $ DOCKER_EXIT_CODE=$?
#
# Signature:
#   $ dn::execute_compose <docker-compose.yaml> [<optional flag>] [-- <any docker cmd+arg>]
#
# Arguments:
#   see function print_help_in_terminal or execute the script with the --help flag
#
# Note:
#   Dont use "set -e" in this script as it will affect the build system policy, use the --fail-fast flag instead
#
# =================================================================================================
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

# Variable set for export
declare -x BUILDKIT_PROGRESS
declare -x REPOSITORY_VERSION
#declare -x BASE_IMAGE
#declare -x OS_NAME
#declare -x BASE_IMG_TAG_PREFIX
declare -x DEPENDENCIES_BASE_IMAGE
declare -x TAG_OS_VERSION
declare -x DEPENDENCIES_BASE_IMAGE_TAG
declare -x DN_IMAGE_TAG
declare -x PROJECT_TAG
declare -x ROS_DISTRO
declare -x ROS_PKG
declare -x DN_GLOBAL_CONFIG
declare -a COMPOSE_FILE_OVERRIDE_FLAG
declare -x DN_TARGET_DEVICE
declare -x DN_COMPOSE_PLATFORMS


function dn::show_debug_build_information() {
  echo -e "\n==============================================================================================="
  echo -e "===Debug breakpoint============================================================================\n"
  tree -a -L 1 "${DN_PATH}"
  echo -e "\n==============================================================================================="
  echo -e "===============================================================================================\n"
  tree -a -L 3 "${DN_PATH}"
  echo -e "\n==============================================================================================="
  echo -e "===============================================================================================\n"
  printenv | grep \
      -e DN_IMAGE_TAG=* \
      -e DN_PROJECT_GID=* \
      -e DN_PROJECT_DEPLOY_REPO_BRANCH=* \
      -e DN_PROJECT_COMPOSE_NAME=* \
      -e DN_PROJECT_UID=* \
      -e DN_CONTAINER_NAME=* \
      -e DN_SPLASH_NAME=* \
      -e DN_PROJECT_BASE_IMG=* \
      -e DN_PROJECT_HUB=* \
      -e DN_PROJECT_IMAGE_NAME=* \
      -e DN_SRC_NAME=* \
      -e DN_COMPOSE_PLATFORMS=* \
      -e DN_GIT_NAME=* \
      -e DN_IMPORTED=* \
      -e DN_PROMPT_NAME=* \
      -e DN_PROJECT_USER=* \
      -e DN_PATH=* \
      -e DN_PROJECT_GIT_NAME=* \
      -e DN_TARGET_DEVICE=* \
      -e DN_GIT_REMOTE_URL=* \
      -e DN_HUB=* \
      -e DN_PROJECT_GIT_DOMAIN=* \
      -e DN_IMAGE_TAG_NO_ROS=* \
      -e IS_TEAMCITY_RUN=*
  echo -e "\n===============================================================================================\n"
  printenv | grep \
      -e TEAMCITY_GIT_PATH=* \
      -e TEAMCITY_BUILD_PROPERTIES_FILE=* \
      -e TEAMCITY_GIT_VERSION=* \
      -e TEAMCITY_CAPTURE_ENV=* \
      -e TEAMCITY_PROCESS_PARENT_FLOW_ID=* \
      -e TEAMCITY_PROCESS_FLOW_ID=* \
      -e TEAMCITY_BUILDCONF_NAME=* \
      -e TEAMCITY_VERSION=* \
      -e TEAMCITY_PROJECT_NAME=*
  echo -e "\n===============================================================================================\n"
  printenv | grep \
      -e NBS_TMP_TEST_LIB_SOURCING_ENV_EXPORT=* \
      -e NBS_BUILD_MATRIX_CONFIG=* \
      -e NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE=* \
      -e NBS_GIT_REMOTE_URL=* \
      -e NBS_GIT_NAME=* \
      -e NBS_COMPOSE_DIR=* \
      -e NBS_SRC_NAME=* \
      -e NBS_PATH=* \
      -e NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG=* \
      -e NBS_SPLASH_NAME=* \
      -e NBS_PROMPT_NAME=* \
      -e NBS_IMPORTED=*
  echo -e "\n===============================================================================================\n"
  printenv | grep \
      -e N2ST_GIT_NAME=* \
      -e N2ST_PROMPT_NAME=* \
      -e N2ST_PATH=* \
      -e N2ST_SRC_NAME=* \
      -e N2ST_GIT_REMOTE_URL=*
  echo -e "\n===============================================================================================\n"
  echo "PATH=$(printenv PATH)"
  echo -e "\n===============================================================================================\n"
  echo "PWD=$(printenv PWD)"
  echo "OLDPWD=$(printenv OLDPWD)"
  echo "HOME=$(printenv HOME)"
  echo -e "\n===============================================================================================\n"
  echo "_REPO_ROOT=$(printenv _REPO_ROOT)"
  echo "_PATH_TO_SCRIPT=$(printenv _PATH_TO_SCRIPT)"
  echo -e "\n============================================================================Debug breakpoint==="
  echo -e "===============================================================================================\n"

  # Unmute to let the fct beahave as a breakpoint
#  n2st::print_msg_error_and_exit "debug breakpoint"

  return 0
}

function dn::execute_compose() {
  # ....Positional argument........................................................................
  local COMPOSE_FILE="${1:?'Missing the docker-compose.yaml file mandatory argument'}"
  shift # Remove argument value

  local COMPOSE_FILE_GLOBAL="dockerized-norlab-images/core-images/global/docker-compose.global.yaml"

  # ....Default....................................................................................
  local DOCKER_MANAGEMENT_COMMAND=( compose )
  declare -a DOCKER_COMPOSE_CMD_ARGS  # eg: 'build --no-cache --push' or 'up --build --force-recreate'
  local _CI_TEST=false
  local DOCKER_FORCE_PUSH=false
  unset DOCKER_EXIT_CODE
  local MAIN_DOCKER_EXIT_CODE=0
  local ROS_DISTRO_PKG=none
  unset BASE_IMG_TAG_PREFIX


  # ....Pre-condition..............................................................................
  if [[ ! -f  ".env.dockerized-norlab-build-system" ]]; then
    n2st::print_msg_error_and_exit "'dn::execute_compose' function must be executed from the project root!\n Curent working directory is '$(pwd)'"
  fi

  if [[ ! -f  "${COMPOSE_FILE}" ]]; then
    n2st::print_msg_error_and_exit "'dn::execute_compose' can't find the docker-compose.yaml file '${COMPOSE_FILE}' at $(pwd)"
  fi

  if [[ ! -f ${COMPOSE_FILE_GLOBAL} ]]; then
    n2st::print_msg_error_and_exit "The global compose file ${COMPOSE_FILE_GLOBAL} is unreachable"
  fi

  # ....Load environment variables from file.......................................................
  set -o allexport
  source .env.dockerized-norlab-build-system || exit 1
  source "${N2ST_PATH:?'Variable not set'}"/.env.project || exit 1
  source .env.dockerized-norlab-project  || exit 1
  set +o allexport

  # ....DN functions...............................................................................

  function print_help_in_terminal() {
    echo -e "\n
  \$ dn::execute_compose <docker-compose.yaml> [<optional flag>] [-- <any docker cmd+arg>]
    \033[1m
      <optional argument>:\033[0m
        -h, --help                              Get help
        --dockerized-norlab-version <v1.3.1>    The dockerized-norlab release tag
        --base-image <dustynv/pytorch>          The base image name
        --os-name <ubuntu>                      The name os the OS
        --base-img-tag-prefix <2.1>             The base image prefix of the tag
        --tag-os-version <r35.2.1>              Operating system version, see .env.build_matrix for supported version
                                                Note: L4T container tags (e.g. r35.2.1) should match the L4T version
                                                on the Jetson otherwize cuda driver won't be accessible
                                                (source https://github.com/dusty-nv/jetson-containers#pre-built-container-images )
        --force-push                            Execute docker compose push right after the docker
                                                main command (to use when using buildx docker-container driver)
        --docker-debug-logs                     Set Docker builder log output for debug (i.e.BUILDKIT_PROGRESS=plain)
        --fail-fast                             Exit script at first encountered error
        --ci-test-force-runing-docker-cmd
        --buildx-bake                           (experimental) Use 'docker buildx bake <cmd>' instead of 'docker compose <cmd>'

    \033[1m
      [-- <any docker cmd+arg>]\033[0m                 Any argument passed after '--' will be passed to docker compose as docker
                                                command and arguments (default to '${DOCKER_COMPOSE_CMD_ARGS[*]}').
                                                Note: passing script flag via docker --build-arg can be tricky,
                                                      pass them in the docker-compose.yaml if you experience problem.
  "
  }

  # ....TeamCity service message logic.............................................................
  if [[ ${TEAMCITY_VERSION} ]]; then
    export IS_TEAMCITY_RUN=true
    TC_VERSION="TEAMCITY_VERSION=${TEAMCITY_VERSION}"
  else
    export IS_TEAMCITY_RUN=false
  fi

  n2st::print_msg "IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN} ${TC_VERSION}"

  # ====Begin======================================================================================
  SHOW_SPLASH_EC="${SHOW_SPLASH_EC:-true}"

  if [[ "${SHOW_SPLASH_EC}" == 'true' ]]; then
    n2st::norlab_splash "${NBS_SPLASH_NAME}" "${PROJECT_GIT_REMOTE_URL}"
  fi

  n2st::print_formated_script_header 'dn_execute_compose.bash' "${MSG_LINE_CHAR_BUILDER_LVL2}"


  # ....Script command line flags..................................................................
  while [ $# -gt 0 ]; do

    case $1 in
    --dockerized-norlab-version)
      REPOSITORY_VERSION="${2}"
      shift # Remove argument (--dockerized-norlab-version)
      shift # Remove argument value
      ;;
    --base-image)
      BASE_IMAGE="${2}"
      shift # Remove argument (--base-image)
      shift # Remove argument value
      ;;
    --os-name)
      OS_NAME="${2}"
      shift # Remove argument (--os-name)
      shift # Remove argument value
      ;;
    --base-img-tag-prefix)
      BASE_IMG_TAG_PREFIX="${2}"
      shift # Remove argument (--base-img-tag-prefix)
      shift # Remove argument value
      ;;
    --tag-os-version)
      TAG_OS_VERSION="${2}"
      shift # Remove argument (--tag-os-version)
      shift # Remove argument value
      ;;
    --ros2)
      ROS_FLAG="${2}"
      if [[ ${ROS_FLAG} != none ]]; then
        ROS_DISTRO_PKG="${ROS_FLAG}"
        ROS_DISTRO=$(echo "${ROS_DISTRO_PKG}" | sed 's;\-.*;;')
        ROS_PKG=${ROS_DISTRO_PKG/${ROS_DISTRO}-/}
      fi
      shift # Remove argument (--ros2)f
      shift # Remove argument value
      ;;
    --docker-debug-logs)
  #    set -v
  #    set -x
      export BUILDKIT_PROGRESS=plain
      shift # Remove argument (--docker-debug-logs)
      ;;
    --buildx-bake)
      n2st::print_msg_warning "Be advise, the DN --buildx-bake flag is still in developemenmt. Use at your own risk"
      DOCKER_MANAGEMENT_COMMAND=( buildx bake )
      shift # Remove argument (--buildx-bake)
      ;;
    --force-push)
      n2st::print_msg_warning "Be advise, the DN --force-push flag does not support specifiying service manualy via docker command ie docker compose build <my-cool-service>. It will iterate over each service define in the comnpose file."
      DOCKER_FORCE_PUSH=true
      shift # Remove argument (--force-push)
      ;;
    --fail-fast)
      if [[ ${IS_TEAMCITY_RUN} == true ]] ; then
        echo -e "##teamcity[message text='${MSG_BASE_TEAMCITY} Dn --fail-fast flag was set in TC run configuration' status='ERROR']"
        n2st::print_msg_error_and_exit "Be advise, the --fail-fast flag should only be used in local development."
      else
        n2st::print_msg "Be advise, --fail-fast flag in effect"
        set -e
      fi
      shift # Remove argument (--fail-fast)
      ;;
    --ci-test-force-runing-docker-cmd)
      _CI_TEST=true
      shift # Remove argument (--ci-test-force-runing-docker-cmd)
      ;;
    -h | --help)
      print_help_in_terminal
      exit
      ;;
    --) # no more option
      shift
      DOCKER_COMPOSE_CMD_ARGS=( $@ )
      break
      ;;
    *) # Default case
      break
      ;;
    esac

  done

  # ...............................................................................................
  # Note: REPOSITORY_VERSION will be used to fetch the repo at release tag (ref task NMO-252)
  export REPOSITORY_VERSION="${REPOSITORY_VERSION:?'Variable not set, use --help to find the proper flag'}"
  export DEPENDENCIES_BASE_IMAGE="${BASE_IMAGE:?'Variable not set, use --help to find the proper flag'}"
  export DEPENDENCIES_BASE_IMAGE_NAME=$(echo "${BASE_IMAGE}" | sed 's;.*/;;')
  export TAG_OS_VERSION="${TAG_OS_VERSION:?'Variable not set, use --help to find the proper flag'}"

  if [[ -z ${BASE_IMG_TAG_PREFIX} ]]; then
    export DEPENDENCIES_BASE_IMAGE_TAG="${TAG_OS_VERSION}"
    DN_IMAGE_TAG_END="${DEPENDENCIES_BASE_IMAGE_NAME}-${TAG_OS_VERSION}"
  else
    export DEPENDENCIES_BASE_IMAGE_TAG="${BASE_IMG_TAG_PREFIX}-${TAG_OS_VERSION}"
    DN_IMAGE_TAG_END="${DEPENDENCIES_BASE_IMAGE_NAME}-${BASE_IMG_TAG_PREFIX}-${TAG_OS_VERSION}"
  fi

  if [[ ${ROS_FLAG} != none ]]; then
    export ROS_DISTRO="${ROS_DISTRO:?'Variable not set, use --help to find the proper flag'}"
    export ROS_PKG="${ROS_PKG:?'Variable not set, use --help to find the proper flag'}"

    if [[ ${ROS_DISTRO_PKG} != none ]]; then
      DN_IMAGE_TAG_BEGIN="DN-${REPOSITORY_VERSION}-${ROS_DISTRO_PKG/-ros/}"
    else
      DN_IMAGE_TAG_BEGIN="DN-${REPOSITORY_VERSION}"
    fi
  else
    DN_IMAGE_TAG_BEGIN="DN-${REPOSITORY_VERSION}"
  fi
  export DN_IMAGE_TAG="${DN_IMAGE_TAG_BEGIN}-${DN_IMAGE_TAG_END}"

  export PROJECT_TAG="${OS_NAME:?'Variable not set, use --help to find the proper flag'}-${TAG_OS_VERSION}"

  # Note: This is required
  COMPOSE_FILE_OVERRIDE_FLAG=(-f "${COMPOSE_FILE_GLOBAL}")
  if [[ ${OS_NAME} == ubuntu ]]; then
    export DN_TARGET_DEVICE="x86-compute-box"
    export DN_COMPOSE_PLATFORMS="linux/amd64"
    ## /// override main compose file logic ///....................................................
    ## (NICE TO HAVE) ToDo: ref task NMO-514 feat: implement platform selection logic
    ## Note: the proper way is to create an override compose file for each and overload each service
    ##       that get built. Dont override 'docker-compose.global.yaml' as the 'extends' operation
    ##       is executed prior to the 'merging/override' operation.
    # COMPOSE_FILE_OVERRIDE="${COMPOSE_FILE/%build.yaml/override.yaml}"
    # COMPOSE_FILE_OVERRIDE_FLAG+=(-f ${COMPOSE_FILE_OVERRIDE})
    ## export DN_TARGET_DEVICE=multi-arch
    ## ....................................................\\\ override main compose file logic \\\
  elif [[ ${OS_NAME} == l4t ]]; then
    export DN_TARGET_DEVICE="jetson"
    export DN_COMPOSE_PLATFORMS="linux/arm64"
  else
    n2st::print_msg_error_and_exit "OS ${OS_NAME} not matched with any architecture configuration"
  fi

  # ....If defined › execute dn::callback_execute_compose_pre......................................
  NBS_COMPOSE_DIR=$( dirname "$COMPOSE_FILE" )
  if [[ -f "${NBS_COMPOSE_DIR:?err}/dn_callback_execute_compose_pre.bash" ]]; then
    n2st::print_msg "Source and execute ${NBS_COMPOSE_DIR}/dn_callback_execute_compose_pre.bash"
    source "${NBS_COMPOSE_DIR}/dn_callback_execute_compose_pre.bash"
    dn::callback_execute_compose_pre
  fi


  # (CRITICAL) todo:on task end, mute next line ↓↓
  dn::show_debug_build_information


  # ...............................................................................................
  n2st::print_msg "Environment variables set for ${DOCKER_MANAGEMENT_COMMAND[*]}:
    ${MSG_DIMMED_FORMAT}
    REPOSITORY_VERSION=${REPOSITORY_VERSION}
    DEPENDENCIES_BASE_IMAGE=${DEPENDENCIES_BASE_IMAGE}
    TAG_OS_VERSION=${TAG_OS_VERSION}
    DEPENDENCIES_BASE_IMAGE_TAG=${DEPENDENCIES_BASE_IMAGE_TAG}
    ROS_DISTRO=${ROS_DISTRO}
    ROS_PKG=${ROS_PKG}
    DN_IMAGE_TAG=${DN_IMAGE_TAG}
    PROJECT_TAG=${PROJECT_TAG}
    ${MSG_END_FORMAT}"

  if [[ ${IS_TEAMCITY_RUN} == true ]]; then
    # Prevent Teamcity DISPLAY unset warning in build log file
    DISPLAY=${DISPLAY:-':0'} && export DISPLAY
  fi

  if [[ -z ${DOCKER_COMPOSE_CMD_ARGS[*]}  ]]; then
    DOCKER_COMPOSE_CMD_ARGS=(build)
  fi

  n2st::print_msg "Executing docker ${DOCKER_MANAGEMENT_COMMAND[*]} command on ${MSG_DIMMED_FORMAT}${COMPOSE_FILE}${MSG_END_FORMAT} with command ${MSG_DIMMED_FORMAT}${DOCKER_COMPOSE_CMD_ARGS[*]}${MSG_END_FORMAT}"
  n2st::print_msg "Image tag ${MSG_DIMMED_FORMAT}${DN_IMAGE_TAG}${MSG_END_FORMAT}"

  # ...Docker cmd conditional logic................................................................
  ## (☕minor) ToDo: assessment if still usefull >> next bloc ↓↓
  # Note:
  #   - BUILDKIT_CONTEXT_KEEP_GIT_DIR is for setting buildkit to keep the .git directory in the container
  #     Source https://docs.docker.com/build/building/context/#keep-git-directory
#  export BUILDKIT_CONTEXT_KEEP_GIT_DIR=1
  if [[ ${DOCKER_COMPOSE_CMD_ARGS[0]} == build ]] && [[ ${DOCKER_MANAGEMENT_COMMAND[*]} != "buildx bake" ]]; then
    unset DOCKER_COMPOSE_CMD_ARGS[0]
    DOCKER_COMPOSE_CMD_ARGS=( build --build-arg "BUILDKIT_CONTEXT_KEEP_GIT_DIR=1" ${DOCKER_COMPOSE_CMD_ARGS[@]})
  fi

  cd "${DN_PATH:?err}" || exit 1

  # (NICE TO HAVE) ToDo: assessment >> next bloc ↓↓
#  function dn::fetch_target_device() {
#      echo -e "${MSG_EMPH_FORMAT}$( docker compose -f "${COMPOSE_FILE}" "${COMPOSE_FILE_OVERRIDE_FLAG[@]}" config --dry-run | grep -i -e DN_TARGET_DEVICE | sed 's;.*DN_TARGET_DEVICE:;;' | uniq )${MSG_END_FORMAT}"
#  }
#  n2st::print_msg "Target device ›$(dn::fetch_target_device)"

  # ...Execute build and push docker command on each service one at the time.......................
  if [[ ${DOCKER_COMPOSE_CMD_ARGS[0]} == build ]] && [[ ${DOCKER_FORCE_PUSH} == true ]]; then

    local STR_BUILT_SERVICES
    declare -a STR_BUILT_SERVICES=( $( docker compose -f "${COMPOSE_FILE}" "${COMPOSE_FILE_OVERRIDE_FLAG[@]}" config --services --no-interpolate --dry-run) )
    for each_service in ${STR_BUILT_SERVICES[@]}; do
      echo

      if [[ "${each_service}" =~ "global-service-builder-config".* ]] || [[ "${each_service}" =~ "runtime-global-dev-config".* ]]; then
         n2st::print_msg_warning "Skip building ${MSG_DIMMED_FORMAT}${each_service}${MSG_END_FORMAT}"
        :
      else
        n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_CHAR_UTIL}"
        n2st::print_msg "Execute docker build for service ${MSG_DIMMED_FORMAT}${each_service}${MSG_END_FORMAT} and push if image is defined"
        # ...Execute docker command for each service...............................................
        n2st::teamcity_service_msg_blockOpened "Build ${each_service}"
        n2st::show_and_execute_docker "${DOCKER_MANAGEMENT_COMMAND[*]} -f ${COMPOSE_FILE} ${COMPOSE_FILE_OVERRIDE_FLAG[*]} ${DOCKER_COMPOSE_CMD_ARGS[*]} ${each_service}" "$_CI_TEST"
        if [[ ${MAIN_DOCKER_EXIT_CODE} == 0 ]]; then
          # Skip update MAIN_DOCKER_EXIT_CODE if it already failed once
          MAIN_DOCKER_EXIT_CODE="${DOCKER_EXIT_CODE:?"variable was not set by n2st::show_and_execute_docker"}"
          unset DOCKER_EXIT_CODE # ToDo: This is a temporary hack >> delete it when n2st::show_and_execute_docker is refactored using "return DOCKER_EXIT_CODE" instead of "export DOCKER_EXIT_CODE"
        fi
        n2st::teamcity_service_msg_blockClosed "Build ${each_service}"

        # ...Execute PUSH for each service.........................................................
          if [[ "${each_service}" =~ .*'-main' ]] || [[ "${each_service}" =~ .*'-tester' ]]; then
            n2st::print_msg "Skip pushing ${MSG_DIMMED_FORMAT}${each_service}${MSG_END_FORMAT}"
          else
            # ...Force pushing docker images to registry...........................................
            # Note: this is the best workaround when building multi-architecture images across multi-stage
            #       and multi-compose-file as multi-aarch image can't be loaded in the local registry and the
            #       docker compose build --push command is not reliable in buildx builder docker-container driver
            n2st::teamcity_service_msg_blockOpened "Force push ${each_service} image to docker registry"
            export COMPOSE_ANSI=always
            n2st::show_and_execute_docker "compose -f ${COMPOSE_FILE} ${COMPOSE_FILE_OVERRIDE_FLAG[*]} push ${each_service}" "$_CI_TEST"
            if [[ ${MAIN_DOCKER_EXIT_CODE} == 0 ]]; then
              # Skip update MAIN_DOCKER_EXIT_CODE if it already failed once
              MAIN_DOCKER_EXIT_CODE="${DOCKER_EXIT_CODE:?"variable was not set by n2st::show_and_execute_docker"}"
              unset DOCKER_EXIT_CODE # ToDo: This is a temporary hack >> delete it when n2st::show_and_execute_docker is refactored using "return DOCKER_EXIT_CODE" instead of "export DOCKER_EXIT_CODE"
            fi
            n2st::teamcity_service_msg_blockClosed "Force push ${each_service} image to docker registry"
          fi
        fi
    done
  else
    # ....Execute docker command on ALL............................................................
    n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_CHAR_UTIL}"
    STR_TC_SERVICE_MSG="${DOCKER_COMPOSE_CMD_ARGS[0]}ing $( basename "${COMPOSE_FILE}")"
    n2st::teamcity_service_msg_blockOpened "${STR_TC_SERVICE_MSG}"
    n2st::show_and_execute_docker "${DOCKER_MANAGEMENT_COMMAND[*]} -f ${COMPOSE_FILE} ${COMPOSE_FILE_OVERRIDE_FLAG[*]} ${DOCKER_COMPOSE_CMD_ARGS[*]}" "$_CI_TEST"
    if [[ ${MAIN_DOCKER_EXIT_CODE} == 0 ]]; then
      # Skip update MAIN_DOCKER_EXIT_CODE if it already failed once
      MAIN_DOCKER_EXIT_CODE="${DOCKER_EXIT_CODE:?"variable was not set by n2st::show_and_execute_docker"}"
      unset DOCKER_EXIT_CODE # ToDo: This is a temporary hack >> delete it when n2st::show_and_execute_docker is refactored using "return DOCKER_EXIT_CODE" instead of "export DOCKER_EXIT_CODE"
    fi
    n2st::teamcity_service_msg_blockClosed "${STR_TC_SERVICE_MSG}"
  fi

  n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_CHAR_UTIL}"
  echo

  # ....If defined › execute dn::callback_execute_compose_post......................................
  if [[ -f "${NBS_COMPOSE_DIR:?err}/dn_callback_execute_compose_post.bash" ]]; then
    n2st::print_msg "Source and execute ${NBS_COMPOSE_DIR}/dn_callback_execute_compose_post.bash"
    source "${NBS_COMPOSE_DIR}/dn_callback_execute_compose_post.bash"
    dn::callback_execute_compose_post
  fi

  # ....Show feedback..............................................................................
  n2st::print_msg "Environment variables used by compose:\n
  ${MSG_DIMMED_FORMAT}    REPOSITORY_VERSION=${REPOSITORY_VERSION} ${MSG_END_FORMAT}
  ${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE=${DEPENDENCIES_BASE_IMAGE} ${MSG_END_FORMAT}
  ${MSG_DIMMED_FORMAT}    TAG_OS_VERSION=${TAG_OS_VERSION} ${MSG_END_FORMAT}
  ${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE_TAG=${DEPENDENCIES_BASE_IMAGE_TAG} ${MSG_END_FORMAT}
  ${MSG_DIMMED_FORMAT}    ROS_DISTRO=${ROS_DISTRO} ${MSG_END_FORMAT}
  ${MSG_DIMMED_FORMAT}    ROS_PKG=${ROS_PKG} ${MSG_END_FORMAT}
  ${MSG_DIMMED_FORMAT}    DN_IMAGE_TAG=${DN_IMAGE_TAG} ${MSG_END_FORMAT}
  ${MSG_DIMMED_FORMAT}    PROJECT_TAG=${PROJECT_TAG} ${MSG_END_FORMAT}
  "

# (NICE TO HAVE) ToDo: assessment >> next bloc ↓↓
#  n2st::print_msg "Targeted device ›$(dn::fetch_target_device)"

  n2st::print_formated_script_footer 'dn_execute_compose.bash' "${MSG_LINE_CHAR_BUILDER_LVL2}"

  return "${MAIN_DOCKER_EXIT_CODE}"
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  echo -e "\n[${MSG_ERROR_FORMAT}ERROR${MSG_END_FORMAT}] This script must be sourced  i.e.: $ source dn_execute_compose.bash" 1>&2
  exit 1

else
  # This script is being sourced, ie: __name__="__source__"

  if [[ "${DN_IMPORTED}" != "true" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} You need to execute ${MSG_DIMMED_FORMAT}import_dockerized_norlab_tools.bash${MSG_END_FORMAT} before sourcing ${MSG_DIMMED_FORMAT}dn_execute_compose.bash${MSG_END_FORMAT}." 1>&2
    exit 1
  else
    # NBS was imported prior to the script execution
    :
  fi

fi
