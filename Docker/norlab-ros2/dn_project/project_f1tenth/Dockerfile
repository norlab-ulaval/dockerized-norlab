
# ////Project: F1TENTH//////////////////////////////////////////////////////////////////////////////////////////////

ARG  BASE_IMG_TAG=arm64-l4t-r35.2.1
FROM norlabsnow/norlab-ros2-humble-dependencies:${BASE_IMG_TAG} AS dn-project


ARG DN_TARGET_PROJECT_SRC_REPO=NorLab_MPPI
# Note: `DN_TARGET_PROJECT_SRC_REPO` is required by the `/develop/ros2_rebuild_dev_workspace.bash` script
ENV DN_TARGET_PROJECT_SRC_REPO=${DN_TARGET_PROJECT_SRC_REPO}
ARG DS_TARGET_PROJECT_SRC_DOMAIN=RedLeader962
ENV DS_TARGET_PROJECT_SRC_DOMAIN=${DS_TARGET_PROJECT_SRC_DOMAIN}

# ====project custom steps==========================================================================================

# ----F1TENTH gym simulator----------------------------------------------------------------------------------------
FROM dn-project AS f1tenth-simulator
# Steps inspired from https://github.com/f1tenth/f1tenth_gym/blob/main/Dockerfile

ENV LIBGL_ALWAYS_INDIRECT=1
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
    # `all` if `NVIDIA_VISIBLE_DEVICES` is unset or null
    # NVIDIA_VISIBLE_DEVICES is a substitute for `--gpus all` flag
    # see https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#environment-variables-oci-spec


ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    # set to `graphics` or add `graphics` to `NVIDIA_DRIVER_CAPABILITIES,`
    # see https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#driver-capabilities

RUN apt-get update --fix-missing \
#    && apt-get install --assume-yes --no-install-recommends \
    && apt-get install --assume-yes \
#            python3-dev \
#            python3-pip \
#            git \
#            build-essential \
            libgl1-mesa-dev \
#            mesa-utils \
            libglu1-mesa-dev \
            fontconfig \
            libfreetype6-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

#RUN pip3 install --upgrade pip \
#    && pip3 install --no-cache-dir \
#      PyOpenGL \
#      PyOpenGL_accelerate


# ....Custom f1tenth steps..........................................................................................
WORKDIR /
#RUN git clone https://github.com/f1tenth/f1tenth_gym.git
#WORKDIR /f1tenth_gym
#WORKDIR ${DS_DEV_WORKSPACE}/f1tenth_gym
#RUN pip3 install -e .

RUN git clone https://github.com/f1tenth/f1tenth_gym.git  \
    && cd /f1tenth_gym  \
    && pip3 install -e .


## Quick hack to fix `ImportError: Can't find framework /System/Library/Frameworks/OpenGL.framework.`
## See known issues comment https://github.com/f1tenth/f1tenth_gym
# RUN pip3 install pyglet==1.5.11

# ----Python--------------------------------------------------------------------------------------------------------
# See NMO-159 ⚒︎ → `pip install` new module

# ....RL/Control library › torchrl..................................................................................
FROM f1tenth-simulator AS torchrl-dependencies

# Currently, torchrl and tensordict are being developped in parallel. If you're installing torchrl from the git repo,
# you should do the same with tensordict (either via a clone or via
#   `pip install git+https://github.com/pytorch-labs/tensordict`).
#
# Source: https://github.com/pytorch/rl/issues/1050#issuecomment-1506429635

#RUN mkdir /pytorch-labs && cd /pytorch-labs  \
#    && git clone https://github.com/pytorch-labs/tensordict.git  \
#    && cd /pytorch-labs/tensordict && pip install -e .

RUN pip3 install --no-cache-dir \
#  git+https://github.com/pytorch-labs/tensordict@v0.1.0
#  git+https://github.com/pytorch-labs/tensordict@0.0.3
  git+https://github.com/pytorch-labs/tensordict \
    functorch
    # quickfix: reinstalling `functorch` is a hack for the torchRL installation problem

# Sanity check
RUN python -c "import tensordict"

FROM torchrl-dependencies AS rl-lib-torchrl-install
# TorchRL › https://github.com/pytorch/rl

#WORKDIR /torchrl
#RUN git clone https://github.com/pytorch/rl
#WORKDIR /torchrl/rl
#RUN pip3 install -e .

#RUN mkdir /torchrl && cd /torchrl  \
#    && git clone https://github.com/pytorch/rl  \
#    && cd ./rl && pip install -e .

WORKDIR /torchrl
RUN pip3 install --no-cache-dir \
#  git+https://github.com/pytorch/rl@v0.1.0
#  git+https://github.com/pytorch/rl@0.0.3
#    git+https://github.com/pytorch/rl --force-reinstall
    git+https://github.com/pytorch/rl

# Sanity check
WORKDIR "/"
RUN python -c "import torchrl"

FROM rl-lib-torchrl-install AS torchrl-optional-dependencies
#RUN pip3 install --no-cache-dir tensordict


# . . Install torchrl optional dependencies. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# Visualization and MLops
RUN pip3 install --no-cache-dir \
    tensorboard \
    "hydra-core>=1.1" \
    hydra-submitit-launcher \
    wandb
    # Weights & Biases (wandb) › The developer-first MLOps platform
    #   - www: https://wandb.ai
    #   - pypi: https://pypi.org/project/wandb/

## rendering
#RUN pip3 install --no-cache-dir moviepy

## deepmind control suite
#RUN pip3 install --no-cache-dir dm_control

## gym, atari games
#RUN pip3 install --no-cache-dir "gym[atari]" "gym[accept-rom-license]" pygame

# ....RL/Control library............................................................................................
# ....Tools.........................................................................................................
FROM torchrl-optional-dependencies AS rl-lib

RUN pip3 install --no-cache-dir \
    'pytorch-mppi[tune]' \
    mbrl \
    ray \
    tqdm

CMD [ "bash" ]

