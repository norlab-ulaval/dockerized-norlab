
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS main

# ====Begin=============================================================================================
# See https://github.com/f1tenth/f1tenth_gym/blob/main/Dockerfile

# Note: fix for `libGL error: No matching fbConfigs or visuals found`
#       and `libGL error: failed to load driver: swrast` error
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
            libgl1-mesa-dev \
            libglu1-mesa-dev \
            fontconfig \
            libfreetype6-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


## ....F1TENTH install.............................................................................

# Fix for gym <-> F1TENTH TypeError:
# >>> obs, step_reward, done, info = env.reset(np.array([[conf.sx, conf.sy, conf.stheta]]))
# >>>     TypeError: reset() takes 1 positional argument but 2 were given
# Check pull request #35 "reset() function signature no longer works with gym 0.21.0" at https://github.com/f1tenth/f1tenth_gym/issues/35
# Gym version: 0.21.0 -> oct 6 2021; 0.20.0 -> Sep 14, 2021;
#RUN pip3 install --no-cache-dir 'gym[classic_control,box2d]<0.21.0'

## Legacy install step
#RUN pip3 install --no-cache-dir --upgrade pip \
#    && pip3 install --no-cache-dir  \
#      'gym[classic_control,box2d]==0.19.0'

# Note: fix openai gym + f110 gym install error
#   See "ERROR: Failed building wheel for gym" https://github.com/freqtrade/freqtrade/issues/8376#issuecomment-1520844382
RUN pip3 install --upgrade \
        'wheel==0.38.4' \
        'setuptools==65.5.1'

# Notes › f110_gym:
#   - "editable_mode=compat" is a quickhack for the pip editable mode issue related to setuptools
#   - f110_gym need to be installed in editable mode because there is curently no MANIFESTE.in file
#       which prevent some required assets (e.g. maps/) to be included in the install path
WORKDIR /opt
RUN git clone https://github.com/f1tenth/f1tenth_gym.git  \
    && cd ./f1tenth_gym  \
    && pip3 install -e . --config-settings editable_mode=compat


# Quick hack to fix `ImportError: Can't find framework /System/Library/Frameworks/OpenGL.framework.`
# See known issues comment https://github.com/f1tenth/f1tenth_gym
#RUN pip3 install pyglet==1.5.11
#RUN pip3 install pyglet==1.5.20
RUN pip3 install --no-cache-dir --upgrade  \
    'pyglet<1.5.11'


FROM main AS test

RUN echo "Sanity-check" \
    && pip3 show f110_gym \
    && echo \
    && python3 -c "import f110_gym" \
    && python3 -c "from f110_gym.envs import F110Env" \
    && python3 -c "import gym; env = gym.make('f110_gym:f110-v0', num_agents=1, timestep=0.01, ego_idx=0)"

FROM main AS final
WORKDIR ${DN_DEV_WORKSPACE}
CMD [ "bash" ]

