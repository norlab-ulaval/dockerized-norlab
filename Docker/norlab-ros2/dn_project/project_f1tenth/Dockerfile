
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

ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
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

RUN pip3 install --upgrade pip \
    && pip3 install --no-cache-dir \
      PyOpenGL \
      PyOpenGL_accelerate


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

#WORKDIR /pytorch-labs
#RUN git clone https://github.com/pytorch-labs/tensordict.git
#WORKDIR /pytorch-labs/tensordict
#RUN pip3 install -e .

RUN mkdir /pytorch-labs && cd /pytorch-labs  \
    && git clone https://github.com/pytorch-labs/tensordict.git  \
    && cd /pytorch-labs/tensordict && pip3 install -e .

FROM torchrl-dependencies AS rl-lib-torchrl-install

#WORKDIR /torchrl
#RUN git clone https://github.com/pytorch/rl
#WORKDIR /torchrl/rl
#RUN pip3 install -e .

RUN mkdir /torchrl && cd /torchrl  \
    && git clone https://github.com/pytorch/rl  \
    && cd ./rl && pip3 install -e .

FROM rl-lib-torchrl-install AS torchrl-optional-dependencies
#RUN pip3 install --no-cache-dir tensordict

## Sanity check
#WORKDIR "~/"
#RUN python -c "import torch"
#RUN python -c "import torchrl"

# . . Install torchrl optional dependencies. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# diverse
RUN pip3 install --no-cache-dir \
    tensorboard \
    "hydra-core>=1.1" \
    hydra-submitit-launcher \
    wandb

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

