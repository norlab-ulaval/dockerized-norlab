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
#   Read DN_PROJECT_UID
#   Read DN_PROJECT_GID
#   Read DN_PROJECT_PATH
#   Read DN_DEV_WORKSPACE
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
set -e

# (CRITICAL) ToDo: unit-test (ref task NMO-548 and RLRP-213)

# ....Check pre-conditions.........................................................................
{
  test -n "${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'}"
  test -n "${DN_PROJECT_UID:?'Env variable need to be set and non-empty.'}"
  test -n "${DN_PROJECT_GID:?'Env variable need to be set and non-empty.'}"
  test -n "${DN_PROJECT_PATH:?'Env variable need to be set and non-empty.'}"
  test -n "${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'}"
}
# ....Create new user and home.....................................................................
DN_PROJECT_USER_HOME=/home/${DN_PROJECT_USER}

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
}
# ....Simlink and change ownership of project dev workspace........................................
{
  mkdir -p "${DN_PROJECT_USER_HOME}${DN_PROJECT_PATH}/"

  ln -s "${DN_DEV_WORKSPACE}" "${DN_PROJECT_USER_HOME}${DN_DEV_WORKSPACE}/"
  ln -s "${DN_PROJECT_PATH}" "${DN_PROJECT_USER_HOME}${DN_PROJECT_PATH}/"

  # (CRITICAL) ToDo: validate user have permission to execute file in that dir (ref task NMO-548)
  # (CRITICAL) ToDo: assessment >> ownership change should probably go at the last project-develop/Dockerfile or at the dn_entrypoint.init.bash (ref task NMO-548)
  chown -R "${DN_PROJECT_UID}":"${DN_PROJECT_GID}" "${DN_PROJECT_USER_HOME}${DN_PROJECT_PATH}/"
  chown -R "${DN_PROJECT_UID}":"${DN_PROJECT_GID}" "${DN_DEV_WORKSPACE}"
}
# ....Simlink and change ownership of DN container-tools...........................................
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
}

# ....Add all dockerized-norlab accumulated bashrc instruction to project user bashrc..............
(
  echo ""
  echo "# >>> dockerized-norlab bashrc"
  echo "source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bashrc.bash"
  echo ""
) >>"${DN_PROJECT_USER_HOME}/.bashrc"

exit 0
