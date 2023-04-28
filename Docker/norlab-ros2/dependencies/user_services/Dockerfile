
# /// norlab-ros2/user_services ////////////////////////////////////////////////////////////////////////////////////////
# This container is the base image for norlab-ros2 `develop` image and `deploy` image
#
# References:
# - NVIDIA Container Runtime on Jetson: https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson
# - dusty-nv/jetson-containers: https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.noetic
# - ROS noetic installation from source: http://wiki.ros.org/noetic/Installation/Source

# Base image: nvidia linux 4 tegra (L4T) nvidia docker container official image
#   l4t-base: https://ngc.nvidia.com/catalog/containers/nvidia:l4t-base
#   l4t-ros-noetic-pytorch: https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic

#ARG  BASE_IMG_TAG=x86-ubuntu18.04
#ARG  BASE_IMG_TAG=arm64-l4t-r32.6.1
ARG  BASE_IMG_TAG=arm64-l4t-r35.2.1
FROM norlabsnow/norlab-ros2-humble-dependencies-wo-services:${BASE_IMG_TAG} AS services


# ===Service: ssh server================================================================================================

# install development utilities
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        openssh-server \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# ...Setup ssh server...................................................................................................
WORKDIR ${DS_DEV_WORKSPACE}

# ssh port, remaped from default 22 to 2222
ARG DS_PYCHARM_DEV_SERVER_PORT=2222
ENV DS_PYCHARM_DEV_SERVER_PORT=${DS_PYCHARM_DEV_SERVER_PORT}
EXPOSE ${DS_PYCHARM_DEV_SERVER_PORT}

# Inspired from https://austinmorlan.com/posts/docker_clion_development/
RUN ( \
    echo "LogLevel DEBUG2"; \
    echo "PermitRootLogin yes"; \
    echo "PasswordAuthentication yes"; \
    echo "Port ${DS_PYCHARM_DEV_SERVER_PORT}"; \
    echo "Subsystem sftp /usr/lib/openssh/sftp-server"; \
  ) > /etc/ssh/sshd_config_dockerized_snow_openssh_server \
  && mkdir /run/sshd

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

## gdbserver port
#EXPOSE 7777

# ...Add new user.......................................................................................................
ARG NEW_USER=pycharm-debugger
ENV DS_PYCHARM_DEV_USER=${NEW_USER}
ARG PASSWORD=lasagne
RUN useradd -m ${NEW_USER} \
  && yes ${PASSWORD} | passwd ${NEW_USER}
# Add the 'video' groups to new user as it's required for GPU access.
# (not a problem on norlab-og but mandatory on Jetson device)
# Ref: https://forums.developer.nvidia.com/t/how-to-properly-create-new-users/68660/2
RUN usermod -a -G video ${NEW_USER}

# ...root config........................................................................................................
# user:newpassword
RUN echo "root:${PASSWORD}" | chpasswd

# (Optional) Change default shell for new user
#RUN usermod -s /bin/bash ${NEW_USER}



# ===Service: dockerized-norlab customized prompt=========================================================================

# ...Configure prompt...................................................................................................
# ♜ Doc: https://powerline.readthedocs.io

# Install Powerline requirement
# Be advise, dont install `xrandr` it's not compatible with the Jetson kernel.
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        socat \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --verbose psutil
# RUN pip3 install --no-cache-dir --verbose pygit2  # Not required by Powerline if Git is installed

# Require a Vim installation suporting python
#       $ vim --version | grep +python
#       +python/dyn +python3/dyn
# 👍🏼 if it print either +python or +python3


# . . Install Powerline + special font pack. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  
# Note: If Powerline prompt fonts are missing in PyCharm terminal, check that the `Console Scheme` / `Consol Font` /  `Use console font instead of default` is set to `Font:` `MesloLGS NF` (!)

## Ref: https://github.com/powerline/fonts
RUN pip3 install --no-cache-dir --verbose powerline-status
RUN apt-get update \
    && apt-get install -y --no-install-recommends fonts-powerline \
    && rm -rf /var/lib/apt/lists/*

#RUN /bin/bash -c "if [[ ${DN_ACTIVATE_POWERLINE_PROMT} == 'true' ]]; then \
#        pip3 install --no-cache-dir --verbose powerline-status \
#        && apt-get update \
#            && apt-get install -y --no-install-recommends fonts-powerline \
#            && rm -rf /var/lib/apt/lists/*; \
#    else \
#      echo 'Skip Powerline prompt configuration'; \
#    fi"
##      exit 1; \



# . . Config bash shell. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# Fetched Powerline install location using:
#    $(pip3 show powerline-status | grep Location: | sed 's/Location: //g')/powerline

## (Priority) ToDo: Change hardcoded version to dynamic version
## see https://github.com/moby/moby/issues/29110
#ARG TMP_PV=$(python --version)
#ENV DS_PYTHON3_VERSION="python${TMP_PV:-7:-3}"
ENV DS_PYTHON3_VERSION="python3.8"

# Note: regarding the `${TMP_PV:-7:-3}` docker notation instead of the unix notation ${TMP_PV:7:3}, see post
#       - How to use combination of ARG and ENV in Dockerfile to prevent unsupported modifier in substitution?: https://stackoverflow.com/questions/68935923/how-to-use-combination-of-arg-and-env-in-dockerfile-to-prevent-unsupported-modif
#       - https://docs.docker.com/engine/reference/builder/#using-arg-variables

ARG DN_ACTIVATE_POWERLINE_PROMT='true'
ENV DN_ACTIVATE_POWERLINE_PROMT="${DN_ACTIVATE_POWERLINE_PROMT}"

ENV POWERLINE_DIR=/usr/local/lib/${DS_PYTHON3_VERSION}/dist-packages/powerline
RUN ( \
    ## Note: powerline-daemon and powerline10k are os specific app.
#    echo "if [ -f `which powerline-daemon` ]; then"; \
    ## Note: Use $"VAR" instead of "$VAR" to write the $VAR in the bashrc without substitution
#    echo "if [[ '$DN_ACTIVATE_POWERLINE_PROMT' == 'true' ]]; then"; \
    echo "if [[ $"DN_ACTIVATE_POWERLINE_PROMT" == 'true' ]]; then"; \
    echo "  # Start the powerline daemon quietly"; \
    echo "  powerline-daemon --quiet"; \
    echo "  # Set Powerline env var for bash"; \
    echo "  POWERLINE_BASH_CONTINUATION=1"; \
    echo "  POWERLINE_BASH_SELECT=1"; \
    echo "  # Invoke binding script"; \
    echo "  . ${POWERLINE_DIR}/bindings/bash/powerline.sh"; \
    echo "else "; \
    echo "  # Skip or stop Powerline prompt configuration"; \
    echo "  powerline-daemon --quiet --kill"; \
#    echo "  export PS1='❆  \e[0;34m$"{DN_CONTAINER_NAME}"\e[0m  \w\$ '"; \
#    echo "  export PS1='\033[1;34m❆  $"{DN_CONTAINER_NAME}" \033[0m\033[2m \w \033[0m\033[1;34m\033[0m '"; \
#    echo "  export PS1='\033[1;37m❆ \033[0m\033[2m \w \033[0m\033[1;37m\033[0m '"; \
#    echo "  export PS1='\033[1;37m❆ ›\033[0m\033[2m \w\033[0m\033[1;37m\$\033[0m '"; \
#    echo "  export PS1='\033[1;37m❆ \033[0m\033[2m \w\033[0m\033[1;37m\$\033[0m '"; \
    ## Note: `\[ .. \]` is required to escape the sequence lenght from the prompt line lenght as there is a limit  \
    ##  https://unix.stackexchange.com/questions/71007/how-to-customize-ps1-properly
    echo '  export PS1="\[\033[1;37m❆ ❯\033[0m\033[2m \$``{DN_CONTAINER_NAME}`` \033[0m\033[1;37m❯\033[0m\033[2m \w\033[0m\033[1;37m\$\033[0m \]"'; \
    echo "fi"; \
  ) >> ~/.bashrc

# echo "\033[1;37m BlaBla \033[0m"

WORKDIR $POWERLINE_DIR

COPY ./prompt/config_files/config.json config_files/
COPY ./prompt/config_files/themes/shell/dockerized_snow.json config_files/themes/shell/dockerized_snow.json
COPY ./prompt/config_files/colorschemes/shell/dockerized_snow.json config_files/colorschemes/shell/dockerized_snow.json

# (Priority) todo:on task end >> delete bloc line
#RUN echo $POWERLINE_DIR
#RUN /bin/bash -c "cd config_files && tree -L 3"

# Set Promt logo
#RUN echo DS_LOGO=$'\u22EF𝐒\u22EF' >> ~/.bashrc
#RUN export DS_LOGO=$'\U22EF\U1D412\U22EF'  # ⋯𝐒⋯        mathematical bold capital s (U+1D412)
#RUN export DS_LOGO=$'\uf2dc' >> ~/.bashrc  #            nf-fa-snowflake_o
#RUN export DS_LOGO=$'\U1D516' >> ~/.bashrc # 𝔖         mathematical fraktur capital s U+1D516      <--
#RUN export DS_LOGO=$'\U1D57E' >> ~/.bashrc # 𝕾         mathematical bold fraktur capital s U+1D57E <--
ENV DN_CONTAINER_NAME=IamSnow


# ===Final build step===================================================================================================
FROM services AS final

WORKDIR /
#RUN echo $PWD
COPY ./tools/dn_info.bash /
#RUN cd / && tree -L 1

RUN ( \
  echo ""; \
  echo "# dockerized-norlab aliases (from dependencies img)"; \
  echo "alias dn_info='bash /dn_info.bash'"; \
  echo ""; \
) >> ~/.bashrc

# ...source & check ROS environement variable...........................................................................
# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
# Check the ROS_PACKAGE_PATH environment variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser/ros2_ws/src:/opt/ros/melodic/share
WORKDIR ${DS_DEV_WORKSPACE}

# Note: ROS2 setup.bash location is in `${ROS_DISTRO}/install/` instead of ROS1 in `${ROS_DISTRO}/`
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash \
    && source ${DS_DEV_WORKSPACE}/install/setup.bash \
    && printenv | grep -e AR_ -e ROS -e MASTER -e HOSTNAME -e DS_ -e DN_" # WE ARE HERE <-- (!)


CMD [ "bash" ]
# //////////////////////////////////////////////////////////////////////////////////////// norlab-ros2/user_services ///



