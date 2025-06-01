
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS package-project-src-code

# (CRITICAL) ToDo: on task NMO-584 end >> delete next bloc ↓↓
#ARG IS_TEAMCITY_RUN
#ENV IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:-false}

ARG DN_PROJECT_DEPLOY_REPO_BRANCH
ENV DN_PROJECT_DEPLOY_REPO_BRANCH=${DN_PROJECT_DEPLOY_REPO_BRANCH:?'Build argument needs to be set and non-empty.'}

ENV DN_PROJECT_SERVICE=project-deploy
ENV DN_PROJECT_SERVICE_DIR=/dockerized-norlab/project/${DN_PROJECT_SERVICE}

ARG DN_DEV_TESTS=${DN_DEV_TESTS:-false}

RUN <<EOF
    n2st::print_msg "Pre-condition checks"
    {
        test $(whoami) == "root" && \
        test -n ${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_USER_HOME:?'Env variable need to be set and non-empty.'} && \
        test -d ${DN_PROJECT_USER_HOME} && \
        test -n ${DN_PROJECT_PATH:?'Env variable need to be set and non-empty.'} && \
        test -d ${DN_PROJECT_PATH} && \
        test -n ${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_GIT_NAME:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_GIT_DOMAIN:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_DEPLOY_REPO_BRANCH:?'Env variable need to be set and non-empty.'} ;
    } || exit 1
EOF


RUN echo "(WARNING) PYTHONUNBUFFERED env variable will be set to 0 for deployment as it affect performances"
# Note: Set python to NOT print stdin/sderr in real-time as it affect overall performances
ENV PYTHONUNBUFFERED=0

# ....Clone or checkout project src code...........................................................
# Implementation note: Copying the repository in the container and then checkout is safer for case
# where the repository is private. It circumvents the problematic of leaked secret that can persist
# in build history and cached layer. Since the repository is already cloned on the host machine
# whether it's a personal computer, a robot or a CI server its safer and easier to manage to
# delegate the responsibility of secret management to the user.
WORKDIR "${DN_DEV_WORKSPACE}/src"
COPY --chown=${DN_PROJECT_USER} --from=context-dn-project-local-src-path . ${DN_PROJECT_GIT_NAME}
WORKDIR ${DN_PROJECT_PATH}
# Dev note:
# - DN_DEV_WORKSPACE: /ros2_ws/src
# - DN_PROJECT_PATH: /ros2_ws/src/dockerized-norlab-project-mock
RUN <<EOF
    if [[ "${DN_DEV_TESTS}" == "true" ]]; then
        # Development test to validate that the git checkout operation was succesfull (pre)
        # mock_file.txt only exist on the dev branch
        tree -L 2 -a "${DN_PROJECT_PATH}"
        {
            test "${DN_PROJECT_GIT_NAME}" == "dockerized-norlab-project-mock" && \
            test "${DN_PROJECT_DEPLOY_REPO_BRANCH}" == "dev" && \
            test "$(git branch --show-current)" == "main" && \
            test ! -f "src/mock_file.txt" ;
        } || exit 1
    fi
EOF

RUN <<EOF
    # ....Debug information (pre-checkout)....................................................
    n2st::print_msg "Debug information (pre-checkout)"
    tree -L 2 -aug "${DN_DEV_WORKSPACE}/src"
    cat ${DN_PROJECT_PATH}/.git
    tree -L 2 -aug ${DN_PROJECT_PATH}/.git

    # ....Checkout...........................................................................
    n2st::print_msg "git status"
    git status

    n2st::print_msg "List repository branches local and remote"
    git branch --list --all

    n2st::print_msg "List repository tags"
    git tag --list

    n2st::print_msg "Checking out branch ${DN_PROJECT_DEPLOY_REPO_BRANCH}"
    git checkout "${DN_PROJECT_DEPLOY_REPO_BRANCH}"

    n2st::print_msg_warning "Note on git checkout faillure: If you experience problem checking out a tag, use prefix 'tags/<my-tags-name>'
    e.g.: DN_PROJECT_DEPLOY_REPO_BRANCH=\"tags/v0.0.1\" in your .dockerized_norlab_project/configuration/.env file"

    # ....Debug information (post-checkout)....................................................
    n2st::print_msg "Debug information (post-checkout)"
    tree -L 2 -aug "${DN_DEV_WORKSPACE}/src"
EOF

RUN <<EOF
    if [[ "${DN_DEV_TESTS}" == "true" ]]; then
        # Development test to validate that the git checkout operation was succesfull (post)
        {
            test "$(git branch --show-current)" == "dev" && \
            test -f "src/mock_file.txt" ;
        } || exit 1
    fi

EOF


# ....Project specific ROS setup..................................................................
WORKDIR ${DN_DEV_WORKSPACE}
RUN <<EOF
    rosdep update --rosdistro ${ROS_DISTRO}
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
    COLCON_FLAGS=()
    if [[ ${TARGETPLATFORM:?err} != ${BUILDPLATFORM:?err} ]]; then
      n2st::print_msg "Builder is running in architecture virtualisation"
      COLCON_FLAGS+=("--executor" "sequential")
    else
        n2st::print_msg "Builder is running on native architecture"
        COLCON_FLAGS+=("--symlink-install")
    fi

    COLCON_FLAGS+=(
        "--cmake-clean-cache"
        "--cmake-args" "-DCMAKE_BUILD_TYPE=Release"
        "--event-handlers" "console_direct+"
    )
    n2st::print_msg "COLCON_FLAGS=("${COLCON_FLAGS[*]}")"
    colcon build ${COLCON_FLAGS[@]}
EOF

# ToDo: refactor project-develop/Dockerfile the project-specific-ros-setup stage as a standalone
#script that can be executed by project-deploy (ref task NMO-558)

# ToDo: Move project-develop/dn_ros2_rebuild_dev_workspace.bash and
# project-develop/dn_fetch_ros_env_variables.bash to a dn-project wide dir so that its available on
# all dn-project service (ref task NMO-558)

# ....DN and DN-project entrypoints and utilities................................................

# Note: Its easier and more robust to copy DN files to root and then symlink them in project user.
WORKDIR ${DN_PROJECT_SERVICE_DIR}
COPY dn_entrypoint.init.bash dn_entrypoint.attach.bash ./


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
### (CRITICAL) ToDo: on task end >> mute next bloc ↓↓
#WORKDIR /dockerized-norlab/dockerized-norlab-images/container-tools
#COPY --from=context-dn-container-tools ./dn_info.bash .
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

    n2st::print_msg "Check that ROS source is available in DN_PROJECT_USER user"
    printenv
    {
      test -n "${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" && \
      python -c "import rclpy" ;
    } || exit 1

    n2st::print_msg "ros2 install pkg sanity check"
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



