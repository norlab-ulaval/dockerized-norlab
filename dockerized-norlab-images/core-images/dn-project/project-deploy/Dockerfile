
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS package-project-src-code

ARG DN_PROJECT_DEPLOY_REPO_BRANCH
ENV DN_PROJECT_DEPLOY_REPO_BRANCH=${DN_PROJECT_DEPLOY_REPO_BRANCH:?'Build argument needs to be set and non-empty.'}

ENV DN_PROJECT_SERVICE=project-deploy
ENV DN_PROJECT_SERVICE_DIR=/dockerized-norlab/project/${DN_PROJECT_SERVICE}

ARG DN_DEPLOY_TESTS=${DN_DEPLOY_TESTS:-false}

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

# ....Clone project src code.......................................................................
# Assume the deploy target branch checkout logic (i.e., checkout to DN_PROJECT_DEPLOY_REPO_BRANCH)
# is handled by the calling script i.e., DNA `build.deploy.bash`
ARG BUILDKIT_CONTEXT_KEEP_GIT_DIR=1
# ....Clone or checkout project src code...........................................................
# Implementation note: Copying the repository in the container and then checkout is safer for case
# where the repository is private. It circumvents the problematic of leaked secret that can persist
# in build history and cached layer. Since the repository is already cloned on the host machine
# whether it's a personal computer, a robot or a CI server its safer and easier to manage to
# delegate the responsibility of secret management to the user.
WORKDIR "${DN_DEV_WORKSPACE}/src"
COPY --from=context-dn-project-local-src-path . ${DN_PROJECT_GIT_NAME}

WORKDIR ${DN_PROJECT_PATH}

# Dev note:
# - DN_DEV_WORKSPACE: /ros2_ws/src
# - DN_PROJECT_PATH: /ros2_ws/src/dockerized-norlab-project-mock
RUN <<EOF
    # ....Debug information .......................................................................
    n2st::print_msg "Debug information"
    tree -L 2 -a "${DN_DEV_WORKSPACE}/src"
    echo
    tree -L 3 -a "${DN_PROJECT_PATH}"
    echo

    # ....Fetch git debug information .............................................................
    echo -e "\ncat ${DN_PROJECT_PATH}/.git"
    cat ${DN_PROJECT_PATH}/.git
    echo
    tree -L 2 -a ${DN_PROJECT_PATH}/.git

    if [[ "${DN_DEPLOY_TESTS}" == "true" ]]; then
        # ....Test ................................................................................
        n2st::print_msg "Development test to validate that the setup for testing git checkout operation is ok (pre)."
        test "${DN_PROJECT_GIT_NAME}" == "dockerized-norlab-project-mock" || n2st::print_msg_error_and_exit "${DN_PROJECT_GIT_NAME} != dockerized-norlab-project-mock"
        test "${DN_PROJECT_DEPLOY_REPO_BRANCH}" == "main" || n2st::print_msg_error_and_exit "${DN_PROJECT_DEPLOY_REPO_BRANCH} != main"
        test "$(git branch --show-current)" == "main" || n2st::print_msg_error_and_exit "$(git branch --show-current) != main"
        echo
        tree -L 2 -a -hug "${DN_PROJECT_PATH}"
        echo
    else
        # ....Build time sanity check .............................................................
        test "$( basename $( git remote get-url origin ) .git )" == "${DN_PROJECT_GIT_NAME}" || n2st::print_msg_error_and_exit "'git remote get-url origin' should point to  ${DN_PROJECT_GIT_NAME} but it point to '$( git remote get-url origin )'"
        test "$(git branch --show-current)" == "${DN_PROJECT_DEPLOY_REPO_BRANCH999}" || {
          n2st::print_msg_error "$(git branch --show-current) != ${DN_PROJECT_DEPLOY_REPO_BRANCH}" && \
          n2st::print_msg_error "Note on git checkout faillure: If you experience problem checking out a tag, use prefix 'tags/<my-tags-name>' e.g.: DN_PROJECT_DEPLOY_REPO_BRANCH=\"tags/v0.0.1\" in your .dockerized_norlab_project/configuration/.env file" && \
          exit 1
        }
    fi

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



