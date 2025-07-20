#!/bin/bash
# =================================================================================================
# Execute build matrix on docker compose docker-compose.dn-dependencies.build.yaml
#
# Usage:
#   $ bash dn_execute_compose_over_build_matrix.bash '<.env.build_matrix.*>' [<optional flag>] [-- <any docker cmd+arg>]
#
#   $ bash dn_execute_compose_over_build_matrix.bash '.env.build_matrix.dn-dependencies' -- build --no-cache --push dependencies-core
#
# Arguments:
#   see function print_help_in_terminal or execute the script with the --help flag
#
# Global:
#   Write STR_BUILD_MATRIX_SERVICES_AND_TAGS to build_all.log
#   Read _CI_TEST
#
# Note:
#   Dont use "set -e" in this script as it will affect the build system policy, use the --fail-fast flag instead
#
# =================================================================================================

## Debug flags
#set -v
#set -x

# ....Default......................................................................................
_BUILD_STATUS_PASS=0

declare -a DOCKER_COMPOSE_CMD_ARGS=( build )
declare -a DN_EXECUTE_COMPOSE_SCRIPT_FLAGS=()
declare -a BASE_IMAGE_TAG_PREFIX_FLAG=()
STR_DOCKER_MANAGEMENT_COMMAND="compose"
DOCKER_FORCE_PUSH=false
DOCKER_EXIT_CODE=1

MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

# ....Pre-condition................................................................................
if [[ ! -f ".env.dockerized-norlab-build-system" ]]; then
  echo -e "\n[${MSG_ERROR_FORMAT}DN ERROR${MSG_END_FORMAT}] 'dn_execute_compose_over_build_matrix.bash' script must be executed from the project root!\n Curent working directory is '$(pwd)'" 1>&2
  exit 1
fi

set -o allexport
source .env.dockerized-norlab-build-system || exit 1
echo -e "Loaded ${MSG_DIMMED_FORMAT}.env.dockerized-norlab-build-system${MSG_END_FORMAT}"
set +o allexport


# ....Positional argument..........................................................................
_DOTENV_BUILD_MATRIX="${1:?'Missing the dotenv build matrix file mandatory argument'}"
shift # Remove argument value

if [[ ! -f "${_DOTENV_BUILD_MATRIX}" ]]; then
  echo -e "\n[${MSG_ERROR_FORMAT}DN ERROR${MSG_END_FORMAT}] 'dn_execute_compose_over_build_matrix.bash' can't find dotenv build matrix file '${_DOTENV_BUILD_MATRIX:?err}'" 1>&2
  exit 1
fi

NBS_BUILD_MATRIX_MAIN=".env.build_matrix.main"
if [[ ! -f "${NBS_BUILD_MATRIX_MAIN}" ]]; then
  n2st::print_msg_error_and_exit "'dn_execute_compose_over_build_matrix.bash' can't find dotenv build matrix file in NBS_BUILD_MATRIX_MAIN='${NBS_BUILD_MATRIX_MAIN:?err}'"
fi



# ....Helper function..............................................................................
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  cd "${DN_PATH:?err}"
  source import_dockerized_norlab_tools.bash || exit 1

  cd "${DN_PATH}"

else
  # This script is being sourced, ie: __name__="__source__"

  if [[ ${_CI_TEST} != true ]]; then
    echo -e "\n[${MSG_ERROR_FORMAT}DN ERROR${MSG_END_FORMAT}] Execute this script in a subshell i.e.: $ bash dn_execute_compose_over_build_matrix.bash" 1>&2
    exit 1
  else
    if [[ "${DN_IMPORTED}" != "true" ]]; then
      echo -e "\n${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} You need to execute ${MSG_DIMMED_FORMAT}import_dockerized_norlab_tools.bash${MSG_END_FORMAT} before sourcing ${MSG_DIMMED_FORMAT}dn_execute_compose_over_build_matrix.bash${MSG_END_FORMAT} otherwise run it with bash." 1>&2
      exit 1
    else
      # NBS was imported prior to the script execution
      :
    fi
  fi

fi


# ....Load environment variables from file.........................................................
cd "${DN_PATH}" || exit 1
set -o allexport

n2st::print_msg "Loading ${MSG_DIMMED_FORMAT}.env.dockerized-norlab-project${MSG_END_FORMAT}"
source .env.dockerized-norlab-project || exit 1

n2st::print_msg "Loading main build matrix ${MSG_DIMMED_FORMAT}${NBS_BUILD_MATRIX_MAIN}${MSG_END_FORMAT}"
source "${NBS_BUILD_MATRIX_MAIN:?'The name of the main .env.build_matrix file is missing'}" || exit 1


if [[ -n ${NBS_OVERRIDE_BUILD_MATRIX_MAIN} ]]; then
  # Note: Override values from .env.build_matrix.main
  n2st::print_msg "Loading main build matrix override ${MSG_DIMMED_FORMAT}${NBS_OVERRIDE_BUILD_MATRIX_MAIN}${MSG_END_FORMAT}"
  source "${NBS_OVERRIDE_BUILD_MATRIX_MAIN}"
fi

n2st::print_msg "Loading build matrix ${MSG_DIMMED_FORMAT}${_DOTENV_BUILD_MATRIX}${MSG_END_FORMAT}"
source "${_DOTENV_BUILD_MATRIX}" || exit 1

# (Priority) ToDo: validate that its still usefull now that we have '.env.dockerized-norlab-project'
source "${N2ST_PATH:?'Variable not set'}"/.env.project || exit 1

set +o allexport

# ....DN functions.................................................................................
cd "${DN_PATH}" || exit 1

# Note: The "set -o/+o allexport" are required to fetch the "declare -x <env var>"
set -o allexport
source dockerized-norlab-scripts/build_script/dn_execute_compose.bash || exit 1
set +o allexport

function print_help_in_terminal() {
  echo -e "\n
\$ ${0} '<.env.build_matrix.*>' [<optional flag>] [-- <any docker cmd+arg>]
  \033[1m
    <optional argument>:\033[0m
      -h, --help          Get help
      --dockerized-norlab-version-build-matrix-override latest
                          The Dockerized-NorLab release tag. Override must be a single value
                          (default to array sequence specified in .env.build_matrix)
      --os-name-build-matrix-override l4t
                          The operating system name. Override must be a single value
                          (default to array sequence specified in .env.build_matrix)
      --ubuntu-version-build-matrix-override jammy
                          Named operating system version. Override must be a single value
                          (default to array sequence specified in .env.build_matrix)
      --l4t-version-build-matrix-override r35.2.1
                          Named operating system version. Override must be a single value
                          (default to array sequence specified in .env.build_matrix)
                          Note: L4T container tags (e.g. r35.2.1) should match the L4T version
                          on the Jetson otherwize cuda driver won't be accessible
                          (source https://github.com/dusty-nv/jetson-containers#pre-built-container-images )
      --force-push        Execute docker compose push right after the docker
                          main command (to use when using buildx docker-container driver)
      --docker-debug-logs
                          Set Docker builder log output for debug (i.e.BUILDKIT_PROGRESS=plain)
      --show-dn-debug-build-info
      --fail-fast         Exit script at first encountered error
      --ci-test-force-runing-docker-cmd
      --buildx-bake       (experimental) Use 'docker buildx bake <cmd>' instead of 'docker compose <cmd>'

  \033[1m
    [-- <any docker cmd+arg>]\033[0m                 Any argument passed after '--' will be passed to docker compose as docker
                                              command and arguments (default to '${DOCKER_COMPOSE_CMD_ARGS[*]}')
"
}

# ToDo: refactor out to 'norlab-shell-script-tools' (ref task NMO-582)
function dn::teamcity_service_msg_blockOpened_custom() {
  local THE_MSG=$1
  if [[ ${IS_TEAMCITY_RUN} == true ]]; then
    echo -e "##teamcity[blockOpened name='${MSG_BASE_TEAMCITY} ${THE_MSG}']"
  fi
}

# ToDo: refactor out to 'norlab-shell-script-tools' (ref task NMO-582)
function dn::teamcity_service_msg_blockClosed_custom() {
  local THE_MSG=$1
  if [[ ${IS_TEAMCITY_RUN} == true ]]; then
    echo -e "##teamcity[blockClosed name='${MSG_BASE_TEAMCITY} ${THE_MSG}']"
  fi
}

# ====Begin=========================================================================================
n2st::norlab_splash "${NBS_SPLASH_NAME}" "${PROJECT_GIT_REMOTE_URL}"

n2st::set_is_teamcity_run_environment_variable

n2st::print_formated_script_header 'dn_execute_compose_over_build_matrix.bash' "${MSG_LINE_CHAR_BUILDER_LVL1}"


# ....Script command line flags....................................................................
while [ $# -gt 0 ]; do

  case $1 in
  --dockerized-norlab-version-build-matrix-override)
    unset NBS_MATRIX_REPOSITORY_VERSIONS
    NBS_MATRIX_REPOSITORY_VERSIONS=("$2")
    shift # Remove argument (--dockerized-norlab-version-build-matrix-override)
    shift # Remove argument value
    ;;
  --os-name-build-matrix-override)
    unset NBS_MATRIX_SUPPORTED_OS
    NBS_MATRIX_SUPPORTED_OS=("$2")
    shift # Remove argument (--os-name-build-matrix-override)
    shift # Remove argument value
    ;;
  --l4t-version-build-matrix-override)
    unset NBS_MATRIX_L4T_SUPPORTED_VERSIONS
    NBS_MATRIX_L4T_SUPPORTED_VERSIONS=("$2")
    shift # Remove argument (--l4t-version-build-matrix-override)
    shift # Remove argument value
    ;;
  --ubuntu-version-build-matrix-override)
    unset NBS_MATRIX_UBUNTU_SUPPORTED_VERSIONS
    NBS_MATRIX_UBUNTU_SUPPORTED_VERSIONS=("$2")
    shift # Remove argument (--ubuntu-version-build-matrix-override)
    shift # Remove argument value
    ;;
  --buildx-bake)
    # (NICE TO HAVE) ToDo: finish implement
    n2st::print_msg "dn_execute_compose_over_build_matrix.bash › set --buildx-bake flag"
    DN_EXECUTE_COMPOSE_SCRIPT_FLAGS+=(--buildx-bake)
    STR_DOCKER_MANAGEMENT_COMMAND="buildx bake"
    shift # Remove argument (--buildx-bake)
    ;;
  --docker-debug-logs)
    #    set -v
    #    set -x
    export BUILDKIT_PROGRESS=plain
    shift # Remove argument (--docker-debug-logs)
    ;;
  --show-dn-debug-build-info)
    DN_EXECUTE_COMPOSE_SCRIPT_FLAGS+=(--show-dn-debug-build-info)
    #_SHOW_DN_DEBUG_BUILD_INFO=true
    shift # Remove argument (--show-dn-debug-build-info)
    ;;
  --force-push)
    DN_EXECUTE_COMPOSE_SCRIPT_FLAGS+=(--force-push)
    DOCKER_FORCE_PUSH=true
    shift # Remove argument (--force-push)
    ;;
  --fail-fast)
    {
      if [[ ${IS_TEAMCITY_RUN} == true ]]; then
        echo -e "##teamcity[message text='${MSG_BASE_TEAMCITY} Dn --fail-fast flag was set in TC run configuration' status='ERROR']"
        n2st::print_msg_error_and_exit "Be advise, the --fail-fast flag should only be used in local development."
      else
        n2st::print_msg "Be advise, --fail-fast flag in effect"
        set -e
      fi
    }
    shift # Remove argument (--fail-fast)
    ;;
  --ci-test-force-runing-docker-cmd)
    DN_EXECUTE_COMPOSE_SCRIPT_FLAGS+=(--ci-test-force-runing-docker-cmd)
    {
      if [[ -z $(docker) ]]; then
        function docker() {
          local tmp=$@
          echo "mock-service-one mock-service-two"
          export DOCKER_EXIT_CODE=0
          return 0
        }
      fi
    }
    shift # Remove argument (--ci-test-force-runing-docker-cmd)
    ;;
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  --) # no more option
    shift
    DOCKER_COMPOSE_CMD_ARGS=($@)
    break
    ;;
  *) # Default case
    break
    ;;
  esac

done


# .................................................................................................
_DOTENV_BUILD_MATRIX_STR="$( basename $(dirname "${_DOTENV_BUILD_MATRIX}"))/$( basename "${_DOTENV_BUILD_MATRIX}" )"

n2st::print_msg "Build images specified in ${MSG_DIMMED_FORMAT}${NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE}${MSG_END_FORMAT} following ${MSG_DIMMED_FORMAT}${_DOTENV_BUILD_MATRIX_STR}${MSG_END_FORMAT}"

## Freeze build matrix env variable to prevent accidental override
## Note: declare -r ==> set as read-only, declare -a  ==> set as an array
declare -r NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE=${NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE}
declare -ra NBS_MATRIX_REPOSITORY_VERSIONS=( "${NBS_MATRIX_REPOSITORY_VERSIONS[@]}" )
declare -ra NBS_MATRIX_SUPPORTED_OS=( "${NBS_MATRIX_SUPPORTED_OS[@]}" )
declare -ra NBS_MATRIX_L4T_SUPPORTED_VERSIONS=( "${NBS_MATRIX_L4T_SUPPORTED_VERSIONS[@]}" )
declare -ra NBS_MATRIX_L4T_BASE_IMAGES=( "${NBS_MATRIX_L4T_BASE_IMAGES[@]}" )
declare -ra NBS_MATRIX_UBUNTU_SUPPORTED_VERSIONS=( "${NBS_MATRIX_UBUNTU_SUPPORTED_VERSIONS[@]}" )
declare -ra NBS_MATRIX_UBUNTU_BASE_IMAGES=( "${NBS_MATRIX_UBUNTU_BASE_IMAGES[@]}" )

declare -ra NBS_MATRIX_ROS_DISTRO=( "${NBS_MATRIX_ROS_DISTRO[@]}" )
declare -ra NBS_MATRIX_ROS_PKG=( "${NBS_MATRIX_ROS_PKG[@]}" )

function dn::print_env_var_build_matrix() {
  local SUP_TEXT=$1
  n2st::print_msg "Environment variables ${MSG_EMPH_FORMAT}(build matrix)${MSG_END_FORMAT} $SUP_TEXT:\n
${MSG_DIMMED_FORMAT}    NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE=${NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE} ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    NBS_MATRIX_REPOSITORY_VERSIONS=(${NBS_MATRIX_REPOSITORY_VERSIONS[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    NBS_MATRIX_SUPPORTED_OS=(${NBS_MATRIX_SUPPORTED_OS[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    NBS_MATRIX_L4T_SUPPORTED_VERSIONS=(${NBS_MATRIX_L4T_SUPPORTED_VERSIONS[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    NBS_MATRIX_L4T_BASE_IMAGES=(${NBS_MATRIX_L4T_BASE_IMAGES[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    NBS_MATRIX_UBUNTU_SUPPORTED_VERSIONS=(${NBS_MATRIX_UBUNTU_SUPPORTED_VERSIONS[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    NBS_MATRIX_UBUNTU_BASE_IMAGES=(${NBS_MATRIX_UBUNTU_BASE_IMAGES[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    NBS_MATRIX_ROS_DISTRO=(${NBS_MATRIX_ROS_DISTRO[*]}) ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    NBS_MATRIX_ROS_PKG=(${NBS_MATRIX_ROS_PKG[*]}) ${MSG_END_FORMAT}
"
}

dn::print_env_var_build_matrix "set for ${STR_DOCKER_MANAGEMENT_COMMAND}"

# ====Crawl build matrix===========================================================================
# Note: EACH_DN_VERSION is used for container labeling and to fetch the repo at release tag
for EACH_DN_VERSION in "${NBS_MATRIX_REPOSITORY_VERSIONS[@]}"; do
  dn::teamcity_service_msg_blockOpened_custom "Bloc=${EACH_DN_VERSION}"

  if [[ -z ${NBS_MATRIX_REPOSITORY_VERSIONS[*]} ]] || [[ ! ${NBS_MATRIX_REPOSITORY_VERSIONS} ]]; then
    echo "NBS_MATRIX_REPOSITORY_VERSIONS=${NBS_MATRIX_REPOSITORY_VERSIONS[*]}"
    n2st::print_msg_error_and_exit "Can't crawl Dockerized-NorLab supported version array because it's empty!"
  fi


  for EACH_OS_NAME in "${NBS_MATRIX_SUPPORTED_OS[@]}"; do
    dn::teamcity_service_msg_blockOpened_custom "Bloc=${EACH_OS_NAME}"

    unset CRAWL_OS_VERSIONS
    unset CRAWL_BASE_IMAGES

    if [[ ${EACH_OS_NAME} == 'ubuntu' ]]; then
      CRAWL_OS_VERSIONS=("${NBS_MATRIX_UBUNTU_SUPPORTED_VERSIONS[@]}")
      CRAWL_BASE_IMAGES=("${NBS_MATRIX_UBUNTU_BASE_IMAGES[@]}")
    elif [[ ${EACH_OS_NAME} == 'l4t' ]]; then
      CRAWL_OS_VERSIONS=("${NBS_MATRIX_L4T_SUPPORTED_VERSIONS[@]}")
      CRAWL_BASE_IMAGES=("${NBS_MATRIX_L4T_BASE_IMAGES[@]}")
    else
      n2st::print_msg_error_and_exit "${EACH_OS_NAME} not supported!"
    fi

    if [[ -z ${CRAWL_OS_VERSIONS[*]} ]]; then
      n2st::print_msg_error_and_exit "Can't crawl ${EACH_OS_NAME} supported version array because it's empty!"
    fi

    if [[ -z ${CRAWL_BASE_IMAGES[*]} ]]; then
      n2st::print_msg_error_and_exit "Can't crawl ${EACH_OS_NAME} base images array because it's empty!"
    fi


    for EACH_OS_VERSION in "${CRAWL_OS_VERSIONS[@]}"; do
      dn::teamcity_service_msg_blockOpened_custom "Bloc=${EACH_OS_VERSION}"

      if [[ -z ${NBS_MATRIX_ROS_DISTRO[*]} ]]; then
        n2st::print_msg_error_and_exit "Can't crawl NBS_MATRIX_ROS_DISTRO array because it's empty! Write 'none' if you want to skip ros."
      elif [[ -z ${NBS_MATRIX_ROS_PKG[*]} ]]; then
        n2st::print_msg_error_and_exit "Can't crawl NBS_MATRIX_ROS_PKG array because it's empty! Write 'none' if you want to skip ros."
      fi

      for EACH_ROS_DISTRO in "${NBS_MATRIX_ROS_DISTRO[@]}" ; do
        dn::teamcity_service_msg_blockOpened_custom "Bloc=${EACH_ROS_DISTRO}"

        _SHOW_DN_DEBUG_BUILD_INFO=${_SHOW_DN_DEBUG_BUILD_INFO:-"false"}
        if [[ ${EACH_ROS_DISTRO} != none ]]; then
          # Ubuntu Focal and L4T 35.*.* <-> ROS2 distro combinaison supported by DN:
          # - Ubunu (Jammy|Noble)/L4T 36.*.* <-> ROS2 (Humble|Jazzy|Kilted)
          # - Ubunu focal/L4T 35.*.* <-> ROS2 galactic
          # - Ubunu focal/L4T 35.*.* <-> ROS2 foxy (deprecated but stay in crawler for retro compatibility)
          if [[ "${EACH_OS_VERSION}" == "focal" ]] || [[ "${EACH_OS_VERSION}" =~ "35.".* ]]; then
            if [[ "${EACH_ROS_DISTRO}" != "galactic" ]] && [[ "${EACH_ROS_DISTRO}" != "foxy" ]]; then
              if [[ "${_SHOW_DN_DEBUG_BUILD_INFO}" == "true" ]]; then
                n2st::print_msg_warning "🔻 DEV >> OS version ${EACH_OS_VERSION} <-> ROS2 distro ${EACH_ROS_DISTRO} (SKIP)"
              fi
              continue
            fi
          elif [[ "${EACH_OS_VERSION}" == "jammy" ]] ||  [[ "${EACH_OS_VERSION}" == "noble" ]] || [[ "${EACH_OS_VERSION}" =~ "36.".* ]]; then
            if [[ "${EACH_ROS_DISTRO}" != "humble" ]] && [[ "${EACH_ROS_DISTRO}" != "jazzy" ]] && [[ "${EACH_ROS_DISTRO}" != "kilted" ]]; then
              if [[ "${_SHOW_DN_DEBUG_BUILD_INFO}" == "true" ]]; then
                n2st::print_msg_warning "🔻 DEV >> OS version ${EACH_OS_VERSION} <-> ROS2 distro ${EACH_ROS_DISTRO} (SKIP)"
              fi
              continue
            fi
          fi

          if [[ "${_SHOW_DN_DEBUG_BUILD_INFO}" == "true" ]]; then
            n2st::print_msg_warning "✅ DEV >> OS version ${EACH_OS_VERSION} <-> ROS2 distro ${EACH_ROS_DISTRO} (DO IT)"
          fi
        fi

        for EACH_ROS_PKG in "${NBS_MATRIX_ROS_PKG[@]}" ; do
          dn::teamcity_service_msg_blockOpened_custom "Bloc=${EACH_ROS_PKG}"

          if [[ ${EACH_ROS_DISTRO} == none ]]; then
            DN_EXECUTE_COMPOSE_SCRIPT_FLAGS+=(--ros2 "none")
          elif [[ -z ${EACH_ROS_PKG[*]} ]]; then
            n2st::print_msg_error_and_exit "Can't crawl NBS_MATRIX_ROS_PKG array because it's empty!"
          elif [[ ${EACH_ROS_DISTRO} != none ]]; then
            DN_EXECUTE_COMPOSE_SCRIPT_FLAGS+=(--ros2 "${EACH_ROS_DISTRO}-${EACH_ROS_PKG}")
          fi

          for EACH_BASE_IMAGES in "${CRAWL_BASE_IMAGES[@]}"; do

            # shellcheck disable=SC2034
            SHOW_SPLASH_EC='false'

            # shellcheck disable=SC2001
            EACH_BASE_IMAGE=$(echo "${EACH_BASE_IMAGES}" | sed 's/:.*//')
            # shellcheck disable=SC2001
            EACH_BASE_IMAGE_TAG_PREFIX=$(echo "${EACH_BASE_IMAGES}" | sed 's/.*://')
            unset BASE_IMAGE_TAG_PREFIX_FLAG
            if [[ -n "${EACH_BASE_IMAGE_TAG_PREFIX}" ]]; then
                BASE_IMAGE_TAG_PREFIX_FLAG+=(--base-img-tag-prefix "${EACH_BASE_IMAGE_TAG_PREFIX}")
            fi


            if [[ ${IS_TEAMCITY_RUN} == true ]]; then
              echo -e "##teamcity[blockOpened name='${MSG_BASE_TEAMCITY} execute dn_execute_compose.bash' description='${MSG_DIMMED_FORMAT_TEAMCITY} --dockerized-norlab-version ${EACH_DN_VERSION} --base-image ${EACH_BASE_IMAGE} --os-name ${EACH_OS_NAME} --tag-os-version ${EACH_OS_VERSION} ${DN_EXECUTE_COMPOSE_SCRIPT_FLAGS[*]} -- ${DOCKER_COMPOSE_CMD_ARGS[*]}${MSG_END_FORMAT_TEAMCITY}|n']"
              echo
            fi

            # ....Repository version checkout logic..........................................................
            if [[ "${EACH_DN_VERSION}" != 'latest' ]] && [[ "${EACH_DN_VERSION}" != 'bleeding' ]] && [[ "${EACH_DN_VERSION}" != 'hot' ]]; then
              cd "${DN_PATH:?err}" || exit 1

              if [[ ${IS_TEAMCITY_RUN} == true ]]; then
                # Solution for "error: object directory ... .git/objects does not exist"
                n2st::print_msg "Git fetch all remote"
                git fetch --all --recurse-submodules --unshallow
              fi

              # Note: keep it here as a testing tool
              n2st::print_msg "Git fetch tag list\n$(git tag --list)"

              # Execute if not run in bats test framework
              if [[ -z ${BATS_VERSION} ]]; then
                n2st::print_msg "Execute git checkout"
                git checkout tags/"${EACH_DN_VERSION}"
              else
                n2st::print_msg_warning "Bats test run › skip \"Execute git checkout\""
              fi

              # ....Validate the DN tag correspond to the checkout branch..............................
            elif [[ "${EACH_DN_VERSION}" == 'latest' ]]; then
              if [[ $(git symbolic-ref -q --short HEAD) != main ]]; then
                  n2st::print_msg_error_and_exit "The DN 'latest' tag was set but the current checkout branch is not the 'main' branch."
              fi
            elif [[ "${EACH_DN_VERSION}" == 'bleeding' ]]; then
              if [[ $(git symbolic-ref -q --short HEAD) != dev ]]; then
                  n2st::print_msg_error_and_exit "The DN 'bleeding' tag was set but the current checkout branch is not the 'dev' branch."
              fi
            fi

            n2st::print_msg "Repository curently checkout at › $(git symbolic-ref -q --short HEAD || git describe --all --exact-match)"

            # ....Execute docker command...............................................................

            # shellcheck disable=SC2086
            dn::execute_compose \
              ${NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE} \
              --dockerized-norlab-version "${EACH_DN_VERSION}" \
              --base-image "${EACH_BASE_IMAGE}" \
              --os-name "${EACH_OS_NAME}" \
              "${BASE_IMAGE_TAG_PREFIX_FLAG[@]}" \
              --tag-os-version "${EACH_OS_VERSION}" \
              "${DN_EXECUTE_COMPOSE_SCRIPT_FLAGS[@]}" \
              -- "${DOCKER_COMPOSE_CMD_ARGS[@]}"

            DOCKER_EXIT_CODE=$?


            # ....Collect image tags exported by dn_execute_compose.bash...............................
            if [[ ${DOCKER_EXIT_CODE} == 0 ]]; then
              MSG_STATUS="${MSG_DONE_FORMAT}Pass ${MSG_DIMMED_FORMAT}›"
              MSG_STATUS_TC_TAG="Pass ›"
            else
              MSG_STATUS="${MSG_ERROR_FORMAT}Fail ${MSG_DIMMED_FORMAT}›"
              MSG_STATUS_TC_TAG="Fail ›"
              _BUILD_STATUS_PASS=$DOCKER_EXIT_CODE

              if [[ ${IS_TEAMCITY_RUN} == true ]]; then
                # Fail the build › Will appear on the TeamCity Build Results page
                echo -e "##teamcity[buildProblem description='BUILD FAIL with docker exit code: ${_BUILD_STATUS_PASS}']"
              fi
            fi

            # Collect image tags exported by dn_execute_compose.bash
            # Global: Read 'DN_IMAGE_TAG' env variable exported by dn_execute_compose.bash
            IMAGE_TAG_CRAWLED=( "${IMAGE_TAG_CRAWLED[@]}" "${MSG_STATUS} ${DN_IMAGE_TAG:?"Env variable not set"}" )
            IMAGE_TAG_CRAWLED_TC=( "${IMAGE_TAG_CRAWLED_TC[@]}" "${MSG_STATUS_TC_TAG} ${DN_IMAGE_TAG}" )
            # .........................................................................................

            if [[ ${IS_TEAMCITY_RUN} == true ]]; then
              echo -e "##teamcity[blockClosed name='${MSG_BASE_TEAMCITY} execute dn_execute_compose.bash']"
            fi

          done
          dn::teamcity_service_msg_blockClosed_custom "Bloc=${EACH_ROS_PKG}"
        done
        dn::teamcity_service_msg_blockClosed_custom "Bloc=${EACH_ROS_DISTRO}"
      done
      dn::teamcity_service_msg_blockClosed_custom "Bloc=${EACH_OS_VERSION}"
    done
    dn::teamcity_service_msg_blockClosed_custom "Bloc=${EACH_OS_NAME}"
  done
  dn::teamcity_service_msg_blockClosed_custom "Bloc=${EACH_DN_VERSION}"
done

# ====Show feedback================================================================================
dn::print_env_var_build_matrix "used by ${STR_DOCKER_MANAGEMENT_COMMAND}"

# Fetch and format the compose file NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE services list
STR_BUILT_SERVICES=$( docker compose -f "${NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE}" config --services --no-interpolate --dry-run | sed 's/^/   - /' )

# Format tag list
for tag in "${IMAGE_TAG_CRAWLED[@]}"; do
  STR_IMAGE_TAG_CRAWLED="${STR_IMAGE_TAG_CRAWLED}\n   ${tag}${MSG_END_FORMAT}"
done

# shellcheck disable=SC2001
STR_BUILD_MATRIX_SERVICES_AND_TAGS="Service crawled:
${MSG_DIMMED_FORMAT}
${STR_BUILT_SERVICES}
${MSG_END_FORMAT}
with tag:
${STR_IMAGE_TAG_CRAWLED}"

# Quick hack to export build matrix log to parent script when called via bash sinc we can't source it whitout breacking docker command
( \
  echo ""; \
  echo "STR_BUILD_MATRIX_SERVICES_AND_TAGS=\"$STR_BUILD_MATRIX_SERVICES_AND_TAGS\""; \
  echo ""; \
) > ./dockerized-norlab-scripts/build_script/build_all.log


n2st::print_msg_done "FINAL › Build matrix completed with command
${MSG_DIMMED_FORMAT}
    $ docker ${STR_DOCKER_MANAGEMENT_COMMAND} -f ${NBS_EXECUTE_BUILD_MATRIX_OVER_COMPOSE_FILE} ${DOCKER_COMPOSE_CMD_ARGS[*]}
${MSG_END_FORMAT}
${STR_BUILD_MATRIX_SERVICES_AND_TAGS}"

n2st::print_formated_script_footer 'dn_execute_compose_over_build_matrix.bash' "${MSG_LINE_CHAR_BUILDER_LVL1}"

# ====TeamCity service message=====================================================================
if [[ ${IS_TEAMCITY_RUN} == true ]]; then
  # Tag added to the TeamCity build via a service message
  for tc_build_tag in "${IMAGE_TAG_CRAWLED_TC[@]}"; do
    echo -e "##teamcity[addBuildTag '${tc_build_tag}']"
  done
fi

# ====Teardown=====================================================================================
cd "${DN_PATH}"

# shellcheck disable=SC2086
exit ${_BUILD_STATUS_PASS}
