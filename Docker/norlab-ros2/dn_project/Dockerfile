
# ////Project: F1TENTH//////////////////////////////////////////////////////////////////////////////////////////////

ARG DN_HUB=norlabsnow
ARG DN_BASE_IMG_TAG=humble-pytorch-arm64-l4t-r35.2.1
FROM ${DN_HUB}/norlab-ros-dependencies-user-services:${DN_BASE_IMG_TAG} AS dn-project



# eg: f1tenth_controller, NorLab_MPPI
ARG DN_TARGET_PROJECT_SRC_REPO
ENV DN_TARGET_PROJECT_SRC_REPO=${DN_TARGET_PROJECT_SRC_REPO?err}
# eg: norlab-ulaval, vaul-ulaval
ARG DN_TARGET_PROJECT_SRC_DOMAIN
ENV DN_TARGET_PROJECT_SRC_DOMAIN=${DN_TARGET_PROJECT_SRC_DOMAIN?err}

# ====Project custom steps==========================================================================================

# ----F1TENTH gym simulator----------------------------------------------------------------------------------------
#FROM dn-project AS f1tenth-simulator
## See https://github.com/f1tenth/f1tenth_gym/blob/main/Dockerfile
#
#ENV LIBGL_ALWAYS_INDIRECT=1
#ENV NVIDIA_VISIBLE_DEVICES \
#    ${NVIDIA_VISIBLE_DEVICES:-all}
#    # `all` if `NVIDIA_VISIBLE_DEVICES` is unset or null
#    # NVIDIA_VISIBLE_DEVICES is a substitute for `--gpus all` flag
#    # see https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#environment-variables-oci-spec
#
#ENV NVIDIA_DRIVER_CAPABILITIES \
#    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
#    # set to `graphics` or add `graphics` to `NVIDIA_DRIVER_CAPABILITIES,`
#    # see https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#driver-capabilities
#
#RUN apt-get update --fix-missing \
##    && apt-get install --assume-yes --no-install-recommends \
#    && apt-get install --assume-yes \
#            libgl1-mesa-dev \
#            libglu1-mesa-dev \
#            fontconfig \
#            libfreetype6-dev \
#    && apt-get clean \
#    && rm -rf /var/lib/apt/lists/*
#
#
#WORKDIR /
#RUN git clone https://github.com/f1tenth/f1tenth_gym.git  \
#    && cd /f1tenth_gym  \
#    && pip3 install -e .
#
### Quick hack to fix `ImportError: Can't find framework /System/Library/Frameworks/OpenGL.framework.`
### See known issues comment https://github.com/f1tenth/f1tenth_gym
## RUN pip3 install pyglet==1.5.11
#
## ====RL/Control library============================================================================================
#FROM f1tenth-simulator AS torchrl-lib
## Note: ON HOLD › Both codebase are in beta release -> instalation is problematic and bugy on arch64. Either
##           - it build and pass the sanity check in docker build but break on docker run `import torchrl`
##           - or it build and work in docker run `import torchrl` but break `torch.cuda.is_available()`
#
#### QuickFix:
####   - Fix warning: `Failed to load image Python extension: {e}` cause by torch and torchvision incompatible version,
####     (see torch <-> torchvision compatible pair: https://github.com/pytorch/vision#installation)
##
##
##RUN mkdir /pytorch-labs && cd /pytorch-labs  \
##    && git clone https://github.com/pytorch-labs/tensordict.git  \
##    && cd /pytorch-labs/tensordict && pip3 install -e .
##
##RUN pip3 install --no-cache-dir --upgrade torchvision
##
##RUN mkdir /torchrl && cd /torchrl  \
##    && git clone https://github.com/pytorch/rl  \
##    && cd ./rl && pip3 install -e .
##
###RUN pip3 install --no-cache-dir \
###  git+https://github.com/pytorch-labs/tensordict@v0.1.0 \
###  git+https://github.com/pytorch/rl@v0.1.0
##
###RUN pip3 install --no-cache-dir \
###    git+https://github.com/pytorch-labs/tensordict \
###    git+https://github.com/pytorch/rl
###
###RUN python -c "import torch" \
###    && python -c "import torchvision" \
###    && python -c "import tensordict" \
###    && python -c "import torchrl"

FROM dn-project AS python-libraries

# . . Stable-baselines3 (SB3) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
## Note: install step transfered to the `requirements_manual.txt` file
#RUN pip3 install --no-cache-dir \
#    'stable-baselines3[extra_no_roms,tests,docs]'  \
#    && pip uninstall -y opencv-python  \
#    && pip install opencv-python-headless


# ====Install from requirements file================================================================================
# ToDo: implement (ref task NMO-159 ⚒︎ → `pip install` new module)

COPY requirements_manual.txt /requirements_manual.txt
# Doc › pip install flag: https://pip.pypa.io/en/stable/cli/pip_install/#options
RUN pip3 install --no-cache-dir --verbose -r /requirements_manual.txt  \
    && rm -rf /requirements_manual.txt

## (!) Hack for solving pytest<->pycharm RuntimeError
## `INTERNALERROR> RuntimeError: wrap_controller at 'pytest_runtest_makereport' /usr/local/lib/python3.8/dist-packages/dash/testing/plugin.py:106 did not yield`
## dash==2.8.1 › Solution from https://github.com/plotly/dash/issues/2460
## Version 2.9.0 is responsible for the pytest<->pycharm RuntimeError
RUN pip3 install --no-cache-dir dash[testing]!=2.9.0

FROM python-libraries AS sanity-check

WORKDIR "/"

#RUN python -c "import torch" \
#    && python -c "import torchvision" \
#    && python -c "from pytorch_mppi import MPPI" \
#    && python -c "import mbrl.algorithms.pets as pets" \
#    && python -c "from stable_baselines3 import PPO"

RUN python -c "import torch" \
    && python -c "import torchvision"

CMD [ "bash" ]

