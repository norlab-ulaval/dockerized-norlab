
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS package-project-src-code

ARG DN_PROJECT_DEPLOY_REPO_BRANCH
ENV DN_PROJECT_DEPLOY_REPO_BRANCH=${DN_PROJECT_DEPLOY_REPO_BRANCH:?'Build argument needs to be set and non-empty.'}

ENV DN_PROJECT_SERVICE=project-deploy
ENV DN_PROJECT_SERVICE_DIR=/dockerized-norlab/project/${DN_PROJECT_SERVICE}

ARG BUILDKIT_CONTEXT_KEEP_GIT_DIR=1
ARG DN_DEV_TESTS=${DN_DEV_TESTS:-false}

RUN <<EOF
    n2st::print_msg "Pre-condition checks..."
    {
        test $(whoami) == "root" && \
        test -n ${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_USER_HOME:?'Env variable need to be set and non-empty.'} && \
        test -d ${DN_PROJECT_USER_HOME} && \
        test -n ${DN_PROJECT_PATH:?'Env variable need to be set and non-empty.'} && \
        test -d ${DN_PROJECT_PATH} && \
        test -n ${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'} && \
        test -d ${DN_DEV_WORKSPACE} && \
        test -n ${DN_PROJECT_GIT_NAME:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_GIT_DOMAIN:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_DEPLOY_REPO_BRANCH:?'Env variable need to be set and non-empty.'} && \
        test -n "${DEBIAN_FRONTEND:?'Env variable need to be set and non-empty.'}" && \
        [[ "${DEBIAN_FRONTEND}" == "noninteractive" ]];
    } || n2st::print_msg_error_and_exit "Failed pre-condition checks!"
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
COPY --from=context-dn-project-local-src-path . ${DN_PROJECT_GIT_NAME}
#COPY --chown=${DN_PROJECT_USER} --from=context-dn-project-local-src-path . ${DN_PROJECT_GIT_NAME}

WORKDIR ${DN_PROJECT_PATH}

# Dev note:
# - DN_DEV_WORKSPACE: /ros2_ws/src
# - DN_PROJECT_PATH: /ros2_ws/src/dockerized-norlab-project-mock
RUN <<EOF
    # ....Debug information (pre-checkout).........................................................
    n2st::print_msg "Debug information (pre-checkout)"
    tree -L 2 -a "${DN_DEV_WORKSPACE}/src"
    echo
    tree -L 3 -a "${DN_PROJECT_PATH}"
    echo
    echo -e "\ncat ${DN_PROJECT_PATH}/.git"
    cat ${DN_PROJECT_PATH}/.git
    echo
    tree -L 2 -a ${DN_PROJECT_PATH}/.git

    if [[ "${DN_DEV_TESTS}" == "true" ]]; then
        n2st::print_msg "Development test to validate that the setup for testing git checkout operation is ok (pre)."
        test "${DN_PROJECT_GIT_NAME}" == "dockerized-norlab-project-mock" || n2st::print_msg_error_and_exit "${DN_PROJECT_GIT_NAME} != dockerized-norlab-project-mock"
        test "${DN_PROJECT_DEPLOY_REPO_BRANCH}" == "dev" || n2st::print_msg_error_and_exit "${DN_PROJECT_DEPLOY_REPO_BRANCH} != dev"
        test "$(git branch --show-current)" == "main" || n2st::print_msg_error_and_exit "$(git branch --show-current) != main"
        test ! -f "src/mock_file.txt" || n2st::print_msg_error_and_exit "mock_file.txt should only exist on the dev branch" # ToDo: TNP-47 fix: mock repo dev branch missing file
        echo
        tree -L 2 -a -hug "${DN_PROJECT_PATH}"
        echo
    else
        # ....Sanity check (pre)...................................................................
        test "$( basename $( git remote get-url origin ) .git )" == "${DN_PROJECT_GIT_NAME}" || n2st::print_msg_error_and_exit "'git remote get-url origin' should point to  ${DN_PROJECT_GIT_NAME} but it point to '$( git remote get-url origin )'"
        test "$( basename $( git rev-parse --show-toplevel ) )" == "${DN_PROJECT_GIT_NAME}" || n2st::print_msg_error_and_exit "'git rev-parse --show-toplevel' should point to  ${DN_PROJECT_GIT_NAME} but it point to '$( git rev-parse --show-toplevel )'"
    fi

EOF

RUN <<EOF
    # ....Checkout.................................................................................
    n2st::print_msg "git status"
    git status || exit 1

    n2st::print_msg "List repository branches local and remote"
    git branch --list --all || exit 1

    n2st::print_msg "List repository tags"
    git tag --list || exit 1

    n2st::print_msg "Checking out branch ${DN_PROJECT_DEPLOY_REPO_BRANCH}"
    git checkout "${DN_PROJECT_DEPLOY_REPO_BRANCH}" || exit 1

    n2st::print_msg_warning "Note on git checkout faillure: If you experience problem checking out a tag, use prefix 'tags/<my-tags-name>'
    e.g.: DN_PROJECT_DEPLOY_REPO_BRANCH=\"tags/v0.0.1\" in your .dockerized_norlab_project/configuration/.env file"

EOF

RUN <<EOF

    # ....Debug information (post-checkout)........................................................
    n2st::print_msg "Debug information (post-checkout)"
    tree -L 2 -aug "${DN_PROJECT_PATH}"

    if [[ "${DN_DEV_TESTS}" == "true" ]]; then
        n2st::print_msg "Development test to validate that the setup for testing git checkout operation is ok (post)."
        test "$(git branch --show-current)" == "dev" || n2st::print_msg_error_and_exit "$(git branch --show-current) != dev"
        test -f "src/mock_file.txt" || n2st::print_msg_error_and_exit "mock_file.txt should exist on the dev branch" # ToDo: TNP-47 fix: mock repo dev branch missing file
        echo
        tree -L 2 -a -hug "${DN_PROJECT_PATH}"
        echo
    else
        # ....Sanity check (post)..................................................................
        test "$(git branch --show-current)" == "${DN_PROJECT_DEPLOY_REPO_BRANCH}" || n2st::print_msg_error_and_exit "$(git branch --show-current) != "${DN_PROJECT_DEPLOY_REPO_BRANCH}""
    fi

EOF


# ....Project specific ROS setup..................................................................
WORKDIR ${DN_DEV_WORKSPACE}
ARG TARGETPLATFORM
ARG BUILDPLATFORM

RUN <<EOF
    apt-get update

    test -n "$( declare -f dn::source_ros2_underlay_only )" || { echo -e "\033[1;31m[DN error]\033[0m The DN lib is not loaded!" 1>&2 && exit 1; }
    dn::source_ros2_underlay_only || exit 1

    n2st::print_msg "Execute rosdep update..."
    rosdep update --rosdistro ${ROS_DISTRO} || n2st::print_msg_error_and_exit "Failed rosdep update!"
    rosdep fix-permissions

    n2st::print_msg "Execute rosdep install..."
    rosdep install \
        --ignore-packages-from-source \
        --from-path ./src  \
        --rosdistro ${ROS_DISTRO} \
        -q \
        -y \
    || n2st::print_msg_error_and_exit "Failed rosdep install!"

    COLCON_FLAGS=()
    if [[ "${TARGETPLATFORM:?err}" != "${BUILDPLATFORM:?err}" ]]; then
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

    n2st::print_msg "Execute colcon build..."
    colcon build ${COLCON_FLAGS[@]} || n2st::print_msg_error_and_exit "Failed colcon build!"

    # ....Teardown.................................................................................
    apt-get autoremove --assume-yes
    apt-get clean
    rm -rf /var/lib/apt/lists/*
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
      # Collect debug information on faillure
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
      echo "alias dn-info='source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash'"
      echo "alias dn-source-ros2='dn::source_ros2'"
      echo "alias dn-source-ros2-underlay-only='dn::source_ros2_underlay_only'"
      echo "alias dn-source-ros2-overlay-only='dn::source_ros2_overlay_only'"
      echo "# <<< dockerized-norlab project-deploy"
      echo ""
    ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash
EOF


FROM package-project-src-code AS test

# ....Test project user............................................................................
USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/usr/local/bin/bash-dn-buildtime", "-c"]

RUN <<EOF
    echo -e "\nSanity check...\n"
    cat ~/.bashrc
    echo
    test -n "$( declare -f dn::source_ros2 )" || { echo -e "\033[1;31m[DN error]\033[0m The DN lib is not loaded!" 1>&2 && exit 1; }
    dn::source_ros2 || exit 1

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

    echo
    n2st::print_msg "ros2 install pkg sanity check"
    {
        ros2 pkg list && \
        test -n "$(ros2 pkg list | grep -e ros_core)" && \
        test -n "$(ros2 pkg list | grep -e rclpy)";
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
SHELL ["/usr/local/bin/bash-dn-buildtime", "-c"]
RUN <<EOF
    echo "Rebuilding rosdep db for project user ${DN_PROJECT_USER}..."
    rosdep update --rosdistro ${ROS_DISTRO:?err} || exit 1
EOF

# Switch to default shell
SHELL ["/bin/bash", "-c"]

# Unset DEBIAN_FRONTEND (which is set globaly in base DN images) before run stage.
ENV DEBIAN_FRONTEND=""

#WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
WORKDIR ${DN_PROJECT_USER_HOME:?'environment variable is not set'}
ENTRYPOINT [ "/dockerized-norlab/project/project-deploy/dn_entrypoint.init.bash" ]
CMD [ "bash" ]



