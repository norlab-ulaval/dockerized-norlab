
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS package-project-src-code

ARG IS_TEAMCITY_RUN
ENV IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:-false}

ARG DN_PROJECT_DEPLOY_REPO_BRANCH
ENV DN_PROJECT_DEPLOY_REPO_BRANCH=${DN_PROJECT_DEPLOY_REPO_BRANCH:?'Build argument needs to be set and non-empty.'}

RUN echo "Pre-condition checks" && {  \
        test $(whoami) == "root" &&  \
        test -n ${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'} && \
        test -d /home/${DN_PROJECT_USER} && \
        test -d ${DN_PROJECT_PATH} && \
        test -n ${DN_PROJECT_GIT_NAME:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_GIT_DOMAIN:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_DEPLOY_REPO_BRANCH:?'Env variable need to be set and non-empty.'} ; \
    } || exit 1


RUN echo "(WARNING) PYTHONUNBUFFERED env variable will be set to 0 for deployment as it affect performances"
# Note: Set python to NOT print stdin/sderr in real-time as it affect overall performances
ENV PYTHONUNBUFFERED=0

# ....Clone or checkout project src code...........................................................
WORKDIR ${DN_DEV_WORKSPACE}/src
RUN <<EOT
echo "\nCheckout ${DN_PROJECT_GIT_NAME} at ${DN_PROJECT_DEPLOY_REPO_BRANCH} branch"

echo -e "\nLog relevant debug information on failure\n"
tree -L 2 -a ${DN_DEV_WORKSPACE}/src
tree -L 2 -a ${DN_DEV_WORKSPACE}/src/${DN_PROJECT_GIT_NAME}
echo

if [[ -e ${DN_PROJECT_GIT_NAME}/.git  ]]; then
  echo "Repository is already cloned"
  cd ${DN_PROJECT_GIT_NAME}
  git checkout "${DN_PROJECT_DEPLOY_REPO_BRANCH}"
elif [[ -d ${DN_PROJECT_GIT_NAME}  ]]; then
  echo "Directory already exist but is not a git repository, so delete it and clone"
  rm -rf ${DN_PROJECT_GIT_NAME}
  git clone --recurse-submodules --branch "${DN_PROJECT_DEPLOY_REPO_BRANCH}" "https://github.com/${DN_PROJECT_GIT_DOMAIN}/${DN_PROJECT_GIT_NAME}.git"
else
  echo "Clone repository"
  git clone --recurse-submodules --branch "${DN_PROJECT_DEPLOY_REPO_BRANCH}" "https://github.com/${DN_PROJECT_GIT_DOMAIN}/${DN_PROJECT_GIT_NAME}.git"
fi

echo -e "\nLog relevant debug information on failure\n"
tree -L 2 -aug ${DN_DEV_WORKSPACE}/src/${DN_PROJECT_GIT_NAME}
EOT

# ....Project spoecific ROS setup..................................................................

WORKDIR ${DN_DEV_WORKSPACE}
RUN rosdep update --rosdistro ${ROS_DISTRO} --include-eol-distros \
    && rosdep fix-permissions \
    && rosdep install  \
            --ignore-packages-from-source \
            --from-path ./src  \
            --rosdistro ${ROS_DISTRO}  \
            -y \
    && rm -rf /var/lib/apt/lists/*


WORKDIR ${DN_DEV_WORKSPACE}
ARG TARGETPLATFORM
ARG BUILDPLATFORM
RUN echo "sourcing ${DN_DEV_WORKSPACE}/install/setup.bash" \
    && source ${DN_DEV_WORKSPACE}/install/setup.bash \
    && echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash" \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && COLCON_FLAGS=() \
    && if [[ ${TARGETPLATFORM:?err} != ${BUILDPLATFORM:?err} ]]; then \
         echo -e "Builder is running in architecture virtualisation" \
         && COLCON_FLAGS+=(--executor sequential) ; \
      else \
         echo -e "Builder is running on native architecture" \
         && COLCON_FLAGS+=(--symlink-install) ; \
      fi \
    && COLCON_FLAGS+=( \
          --cmake-clean-cache \
          --cmake-args -DCMAKE_BUILD_TYPE=Release \
          --event-handlers console_direct+ \
       ) \
    && echo -e "COLCON_FLAGS=("${COLCON_FLAGS[*]}")" \
    && colcon build ${COLCON_FLAGS[@]}

# ....DN and DN-project entrypoints and utilities................................................

# Note: Its easier and more robust to copy DN files to root and then symlink them in project user.
ENV DN_PROJECT_SERVICE_DIR=/dockerized-norlab/project/project-deploy
WORKDIR ${DN_PROJECT_SERVICE_DIR}
COPY dn_entrypoint.init.bash dn_entrypoint.attach.bash ./

# ToDo: refactor project-develop/Dockerfile the project-specific-ros-setup stage as a standalone
#script that can be executed by project-deploy (ref task NMO-558)

# ToDo: Move project-develop/dn_ros2_rebuild_dev_workspace.bash and
# project-develop/dn_fetch_ros_env_variables.bash to a dn-project wide dir so that its available on
# all dn-project service (ref task NMO-558)

RUN chmod +x dn_entrypoint.init.bash && \
    chmod +x dn_entrypoint.attach.bash

# Create directories
ENV DN_PROJECT_USER_HOME=/home/${DN_PROJECT_USER}
WORKDIR ${DN_PROJECT_USER_HOME}

# Create soft link in the home dir
RUN ln -s ${DN_PROJECT_SERVICE_DIR}/dn_entrypoint.init.bash ${DN_PROJECT_USER_HOME}/dn_entrypoint.init.bash \
    && ln -s ${DN_PROJECT_SERVICE_DIR}/dn_entrypoint.attach.bash ${DN_PROJECT_USER_HOME}/dn_entrypoint.attach.bash

# ....Project directories ownership and simlink....................................................

# (CRITICAL) ToDo: validate user have permission to execute file in that dir (ref task NMO-548)
RUN chown -R $(id -u ${DN_PROJECT_USER}):$(id -g ${DN_PROJECT_USER}) ${DN_PROJECT_USER_HOME}
RUN chown -R $(id -u ${DN_PROJECT_USER}):$(id -g ${DN_PROJECT_USER}) ${DN_PROJECT_PATH}
RUN chown -R $(id -u ${DN_PROJECT_USER}):$(id -g ${DN_PROJECT_USER}) ${DN_DEV_WORKSPACE}

# (CRITICAL) ToDo: Not sure its usefull to symlink those (ref task NMO-548)
RUN ln -s "${DN_DEV_WORKSPACE}" "${DN_PROJECT_USER_HOME}${DN_DEV_WORKSPACE}" && \
    ln -s "${DN_PROJECT_PATH}" "${DN_PROJECT_USER_HOME}${DN_PROJECT_PATH}"


FROM package-project-src-code AS test

# ....Test project user............................................................................
USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/bin/bash", "-c"]

RUN test $(whoami) == "${DN_PROJECT_USER}" && test -d /home/${DN_PROJECT_USER} || exit 1

RUN echo "===Check that ROS source is available in DN_PROJECT_USER user=============" \
    && printenv \
    && echo "" \
    && echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" \
    && python -c "import rclpy" \
    && echo "" \
    && echo "===ros2 install pkg sanity check=================================================" \
    && { \
        test -n "$(ros2 pkg list | grep -e ros_core)" &&  \
        test -n "$(ros2 pkg list | grep -e rclpy)" &&  \
        test -n "$(ros2 pkg list | grep -e ackermann_msgs)" &&  \
        test -n "$(ros2 pkg list | grep -e vesc_msgs)" ; \
    } ||  exit 1

RUN { \
    test -x "${DN_PROJECT_USER_HOME}/dn_entrypoint.init.bash" && \
    test -x "${DN_PROJECT_USER_HOME}/dn_entrypoint.attach.bash" && \
    test -d "${DN_PROJECT_USER_HOME}" && \
    test -d "${DN_PROJECT_PATH}" && \
    test -d "${DN_DEV_WORKSPACE}" ; \
    } || exit 1

# unset entrypoint for running test stage
ENTRYPOINT [ "" ]

FROM package-project-src-code AS final

USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/bin/bash", "-c"]

#WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
WORKDIR ${DN_PROJECT_USER_HOME:?'environment variable is not set'}
ENTRYPOINT [ "/dockerized-norlab/project/project-deploy/dn_entrypoint.init.bash" ]
CMD [ "bash" ]



