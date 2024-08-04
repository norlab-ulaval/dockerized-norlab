
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS package-project-src-code

ARG IS_TEAMCITY_RUN
ENV IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:-false}

ARG DN_PROJECT_DEPLOY_REPO_BRANCH
ENV DN_PROJECT_DEPLOY_REPO_BRANCH=${DN_PROJECT_DEPLOY_REPO_BRANCH:?'Build argument needs to be set and non-empty.'}

ENV DN_PROJECT_SERVICE=project-deploy
ENV DN_PROJECT_SERVICE_DIR=/dockerized-norlab/project/${DN_PROJECT_SERVICE}

RUN <<EOF
    echo "Pre-condition checks"
    {
        test $(whoami) == "root" && \
        test -n ${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_USER_HOME:?'Env variable need to be set and non-empty.'} && \
        test -d ${DN_PROJECT_USER_HOME} && \
        test -n ${DN_PROJECT_PATH:?'Env variable need to be set and non-empty.'} && \
        test -d ${DN_PROJECT_PATH} && \
        test -n ${DN_PROJECT_GIT_NAME:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_GIT_DOMAIN:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_DEPLOY_REPO_BRANCH:?'Env variable need to be set and non-empty.'} ;
    } || exit 1
EOF


RUN echo "(WARNING) PYTHONUNBUFFERED env variable will be set to 0 for deployment as it affect performances"
# Note: Set python to NOT print stdin/sderr in real-time as it affect overall performances
ENV PYTHONUNBUFFERED=0

# ....Clone or checkout project src code...........................................................
WORKDIR ${DN_DEV_WORKSPACE}/src
RUN <<EOF
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

    # Sanity check
    test -d ${DN_PROJECT_PATH} || exit 1

    echo -e "\nLog relevant debug information on failure\n"
    tree -L 2 -aug ${DN_DEV_WORKSPACE}/src/${DN_PROJECT_GIT_NAME}
EOF

# ....Project spoecific ROS setup..................................................................

WORKDIR ${DN_DEV_WORKSPACE}
RUN <<EOF
    rosdep update --rosdistro ${ROS_DISTRO} --include-eol-distros
    rosdep fix-permissions
    rosdep install \
        --ignore-packages-from-source \
        --from-path ./src  \
        --rosdistro ${ROS_DISTRO} \
        -y
    rm -rf /var/lib/apt/lists/*
EOF


WORKDIR ${DN_DEV_WORKSPACE}
ARG TARGETPLATFORM
ARG BUILDPLATFORM
RUN <<EOF
    echo "sourcing ${DN_DEV_WORKSPACE}/install/setup.bash"
    source ${DN_DEV_WORKSPACE}/install/setup.bash
    echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash"
    source /opt/ros/${ROS_DISTRO}/setup.bash

    COLCON_FLAGS=()
    if [[ ${TARGETPLATFORM:?err} != ${BUILDPLATFORM:?err} ]]; then
      echo -e "Builder is running in architecture virtualisation"
      COLCON_FLAGS+=("--executor" "sequential")
    else
        echo -e "Builder is running on native architecture"
        COLCON_FLAGS+=("--symlink-install")
    fi

    COLCON_FLAGS+=(
        "--cmake-clean-cache"
        "--cmake-args" "-DCMAKE_BUILD_TYPE=Release"
        "--event-handlers" "console_direct+"
    )
    echo -e "COLCON_FLAGS=("${COLCON_FLAGS[*]}")"
    colcon build ${COLCON_FLAGS[@]}
EOF

# ....DN and DN-project entrypoints and utilities................................................

# Note: Its easier and more robust to copy DN files to root and then symlink them in project user.
WORKDIR ${DN_PROJECT_SERVICE_DIR}
COPY dn_entrypoint.init.bash dn_entrypoint.attach.bash ./

# ToDo: refactor project-develop/Dockerfile the project-specific-ros-setup stage as a standalone
#script that can be executed by project-deploy (ref task NMO-558)

# ToDo: Move project-develop/dn_ros2_rebuild_dev_workspace.bash and
# project-develop/dn_fetch_ros_env_variables.bash to a dn-project wide dir so that its available on
# all dn-project service (ref task NMO-558)

RUN chmod +x dn_entrypoint.init.bash && \
    chmod +x dn_entrypoint.attach.bash

# Create soft link in the home dir
RUN ln -s ${DN_PROJECT_SERVICE_DIR}/dn_entrypoint.init.bash ${DN_PROJECT_USER_HOME}/dn_entrypoint.init.bash \
    && ln -s ${DN_PROJECT_SERVICE_DIR}/dn_entrypoint.attach.bash ${DN_PROJECT_USER_HOME}/dn_entrypoint.attach.bash

# ....Project directories ownership and simlink....................................................

# (CRITICAL) ToDo: validate user have permission to execute file in that dir (ref task NMO-548)
RUN <<EOF
    {
      chown -R $(id -u ${DN_PROJECT_USER:?err}):$(id -g ${DN_PROJECT_USER}) ${DN_PROJECT_USER_HOME:?err}
      chown -R $(id -u ${DN_PROJECT_USER}):$(id -g ${DN_PROJECT_USER}) ${DN_PROJECT_PATH:?err}
      chown -R $(id -u ${DN_PROJECT_USER}):$(id -g ${DN_PROJECT_USER}) ${DN_DEV_WORKSPACE:?err}
    } || {
      # Collect debugg information on faillure
      pwd
      tree -agu
      tree -agu ${DN_PROJECT_USER_HOME}
      tree -agu ${DN_DEV_WORKSPACE}
      exit 1
    }

    echo "Add project-deploy aliases"
    (
      echo ""
      echo "# >>> dockerized-norlab project-deploy"
      echo "# Dockerized-NorLab aliases (from project-deploy img)"
      echo "alias dn_info='source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash'"
      echo "# <<< dockerized-norlab project-deploy"
      echo ""
    ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash
EOF

# ....TMP DEV introspection........................................................................
## (CRITICAL) ToDo: on task end >> mute next bloc ↓↓
#RUN <<EOF
#    # TMP DEV introspection
#    cat /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash
#    echo -e "\n${0}: not implemented error\n" && exit 1
#EOF
# ........................................................................TMP DEV introspection....

# ....TMP DEV dn_info.bash refactor................................................................
## (CRITICAL) ToDo: on task end >> mute next bloc ↓↓
WORKDIR /dockerized-norlab/dockerized-norlab-images/container-tools
COPY --from=context-dn-container-tools ./dn_info.bash .
# ................................................................TMP DEV dn_info.bash refactor....


FROM package-project-src-code AS test

# ....Test project user............................................................................
USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/bin/bash", "-c"]

RUN <<EOF
    {
        test "$(whoami)" == "${DN_PROJECT_USER}" && \
        test -d "/home/${DN_PROJECT_USER}" ;
    } || exit 1

    echo "===Check that ROS source is available in DN_PROJECT_USER user============="
    printenv
    {
      test -n "${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" && \
      python -c "import rclpy" ;
    } || exit 1

    echo "===ros2 install pkg sanity check================================================="
    {
        test -n "$(ros2 pkg list | grep -e ros_core)" && \
        test -n "$(ros2 pkg list | grep -e rclpy)" && \
        test -n "$(ros2 pkg list | grep -e ackermann_msgs)" && \
        test -n "$(ros2 pkg list | grep -e vesc_msgs)" ;
    } ||  exit 1

    {
        test -x "${DN_PROJECT_USER_HOME}/dn_entrypoint.init.bash" && \
        test -x "${DN_PROJECT_USER_HOME}/dn_entrypoint.attach.bash" && \
        test -d "${DN_PROJECT_USER_HOME}" && \
        test -d "${DN_PROJECT_PATH}" && \
        test -d "${DN_DEV_WORKSPACE}" ;
    } || exit 1
EOF

# unset entrypoint for running test stage
ENTRYPOINT [ "" ]

FROM package-project-src-code AS final

USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/bin/bash", "-c"]

#WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
WORKDIR ${DN_PROJECT_USER_HOME:?'environment variable is not set'}
ENTRYPOINT [ "/dockerized-norlab/project/project-deploy/dn_entrypoint.init.bash" ]
CMD [ "bash" ]



