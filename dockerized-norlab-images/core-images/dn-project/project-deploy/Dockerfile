
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS package-project-src-code

ARG DN_PROJECT_DEPLOY_REPO_BRANCH
ENV DN_PROJECT_DEPLOY_REPO_BRANCH=${DN_PROJECT_DEPLOY_REPO_BRANCH:?'Build argument needs to be set and non-empty.'}

#ARG DN_PROJECT_USER
#ENV DN_PROJECT_USER=${DN_PROJECT_USER:?'Build argument needs to be set and non-empty.'}

RUN echo "Pre-condition checks" && {  \
        test $(whoami) == "root" &&  \
        test -n ${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'} && \
        test -d /home/${DN_PROJECT_USER} && \
        test -n ${DN_PROJECT_GIT_NAME:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_GIT_DOMAIN:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_DEPLOY_REPO_BRANCH:?'Env variable need to be set and non-empty.'} ; \
    } || exit 1


RUN echo "(WARNING) PYTHONUNBUFFERED env variable will be set to 0 for deployment as it affect performances"
# Note: Set python to NOT print stdin/sderr in real-time as it affect overall performances
ENV PYTHONUNBUFFERED=0

WORKDIR ${DN_DEV_WORKSPACE}/src
RUN echo "Pull latest ${DN_PROJECT_DEPLOY_REPO_BRANCH} branch" \
    && git clone --branch ${DN_PROJECT_DEPLOY_REPO_BRANCH} https://github.com/${DN_PROJECT_GIT_DOMAIN}/${DN_PROJECT_GIT_NAME}.git  \
    && cd ${DN_DEV_WORKSPACE} \
    && apt-get update --fix-missing \
    && rosdep update --rosdistro ${ROS_DISTRO} --include-eol-distros \
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

# Note: Its easier and more robust to copy DN files to root and then symlink them in project user.
ENV DN_PROJECT_DEVELOP_DIR=/dockerized-norlab/project/project-develop
WORKDIR ${DN_PROJECT_DEVELOP_DIR}
COPY dn_ros2_entrypoint.bash ./
RUN chmod +x ./dn_ros2_entrypoint.bash

ENV DN_PROJECT_USER_HOME=/home/${DN_PROJECT_USER}
WORKDIR ${DN_PROJECT_USER_HOME}

# Create soft link in the home dir
RUN ln -s ${DN_PROJECT_DEVELOP_DIR}/dn_ros2_entrypoint.bash ${DN_PROJECT_USER_HOME}/dn_ros2_entrypoint.bash


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

RUN test -x "${DN_PROJECT_USER_HOME}/dn_ros2_entrypoint.bash"

# unset entrypoint for running test stage
ENTRYPOINT [ "" ]

FROM package-project-src-code AS final

USER ${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}
SHELL ["/bin/bash", "-c"]

WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
COPY --chown=${DN_PROJECT_USER} ../../ ${DN_PROJECT_PATH:?err}/

#WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
WORKDIR ${DN_PROJECT_USER_HOME:?'environment variable is not set'}
ENTRYPOINT [ "/dockerized-norlab/project/project-develop/dn_ros2_entrypoint.bash" ]
CMD [ "bash" ]



