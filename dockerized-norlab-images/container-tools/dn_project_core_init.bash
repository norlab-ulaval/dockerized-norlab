#!/bin/bash
# =================================================================================================
# Dockerized-NorLab project-core image initialization script
# Is executed by '.dockerized_norlab_project/configuration/Dockerfile' in a DN project image
#
# Usage:
#   source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_project_core_init.bash
#
# Globals:
#   Read DN_PROJECT_USER
#   Read DN_PROJECT_USER_HOME
#   Read DN_PROJECT_UID
#   Read DN_PROJECT_GID
#   Read DN_PROJECT_PATH
#   Read DN_DEV_WORKSPACE
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
set -e
pushd "$(pwd)" >/dev/null || exit 1

# (CRITICAL) ToDo: on task NMO-768 end >> delete this line ↓
# source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bashrc_non_interactive.bash

# (CRITICAL) ToDo: unit-test (ref task NMO-548 and RLRP-213)

function dn::initialize_dockerized_norlab_project() {

  # ....Check pre-conditions.......................................................................
  {
    test -n "${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_USER_HOME:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_UID:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_GID:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_PATH:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_GIT_NAME:?'Env variable need to be set and non-empty.'}" ;
  } || n2st::print_msg_error_and_exit "Failed pre-condition check!"

  # ....Create new user and home...................................................................
  # Inspired from https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/
  {
    groupadd --force --gid "${DN_PROJECT_GID}" "${DN_PROJECT_USER}"
    useradd --uid "${DN_PROJECT_UID}" --gid "${DN_PROJECT_GID}" --create-home "${DN_PROJECT_USER}"
    echo "${DN_PROJECT_USER} ALL=(root) NOPASSWD:ALL" >/etc/sudoers.d/"${DN_PROJECT_USER}"
    chmod 0440 "/etc/sudoers.d/${DN_PROJECT_USER}"
    mkdir -p "${DN_PROJECT_USER_HOME}"
    chown -R "${DN_PROJECT_UID}":"${DN_PROJECT_GID}" "${DN_PROJECT_USER_HOME}"

    # Add the 'video' groups to new user as it's required for GPU access.
    # (not a problem on norlab-og but mandatory on Jetson device)
    # Ref: https://forums.developer.nvidia.com/t/how-to-properly-create-new-users/68660/2
    usermod -a -G video,sudo "${DN_PROJECT_USER}"
  } || n2st::print_msg_error_and_exit "Failed new user ${DN_PROJECT_USER} setup!"

  # ....Setup project dev workspace................................................................
  {
    mkdir -p "${DN_PROJECT_PATH}"
    chown -R "${DN_PROJECT_UID}":"${DN_PROJECT_GID}" "${DN_PROJECT_PATH}"

    # Add useful simlink in user root dir
    ln -s "${DN_DEV_WORKSPACE}" "${DN_PROJECT_USER_HOME}/$(basename "${DN_DEV_WORKSPACE}")"
    ln -s "${DN_PROJECT_PATH}" "${DN_PROJECT_USER_HOME}/${DN_PROJECT_GIT_NAME}"
  }

  # ....Simlink and change ownership of DN container-tools.........................................
  {
    DN_CONTAINER_TOOLS_DIR=/dockerized-norlab/dockerized-norlab-images/container-tools
    ln -s "${DN_CONTAINER_TOOLS_DIR}/dn_info.bash" "${DN_PROJECT_USER_HOME}/dn_info.bash"
    ln -s "${DN_CONTAINER_TOOLS_DIR}/import_dockerized_norlab_container_tools.bash" "${DN_PROJECT_USER_HOME}/import_dockerized_norlab_container_tools.bash"
  }

  {
    #cd /dockerized-norlab/dockerized-norlab-images/container-tools
    cd "${DN_PROJECT_USER_HOME}"
    source import_dockerized_norlab_container_tools.bash
    test -n "${DN_PATH:?'Env variable need to be set by import_dockerized_norlab_container_tools.bash and non-empty.'}"
    chown -R "${DN_PROJECT_UID}":"${DN_PROJECT_GID}" "${DN_PATH}"
    git config --system --add safe.directory "*"
  } || exit 1

  # ....Add all dockerized-norlab accumulated bashrc instruction to project user bashrc............
  (
    echo ""
    echo "# >>> dockerized-norlab bashrc"
    echo "source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bashrc.bash"
    echo ""
  ) >>"${DN_PROJECT_USER_HOME}/.bashrc"

  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  dna_error_prefix="\033[1;31m[DNA error]\033[0m"
  echo -e "${dna_error_prefix} This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[N2ST error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }
  dn::initialize_dockerized_norlab_project || n2st::print_msg_error_and_exit "dn::initialize_dockerized_norlab_project exited with error!"
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
