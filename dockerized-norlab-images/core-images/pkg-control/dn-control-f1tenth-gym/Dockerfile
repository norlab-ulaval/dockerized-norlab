
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS main

ARG TARGETPLATFORM
ARG BUILDPLATFORM

# ====Begin=============================================================================================
# See https://github.com/f1tenth/f1tenth_gym/blob/main/Dockerfile

# Note: fix for `libGL error: No matching fbConfigs or visuals found`
#       and `libGL error: failed to load driver: swrast` error
ENV LIBGL_ALWAYS_INDIRECT=1

### (CRITICAL) ToDo: on task NMO-584 end >> delete next bloc ↓↓
#ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
#    # `all` if `NVIDIA_VISIBLE_DEVICES` is unset or null
#    # NVIDIA_VISIBLE_DEVICES is a substitute for `--gpus all` flag
#    # see https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#environment-variables-oci-spec
#
#ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
#    # set to `graphics` or add `graphics` to `NVIDIA_DRIVER_CAPABILITIES,`
#    # see https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#driver-capabilities

RUN <<EOF

    # ....Deb install..............................................................................
    apt-get update

    echo "(deb) Install F110-gym dependencies..."
    apt-get install --assume-yes --no-install-recommends \
            libgl1-mesa-dev \
            libglu1-mesa-dev \
            fontconfig \
            libfreetype6-dev \
      || exit 1

    apt-get autoremove --assume-yes
    apt-get clean
    rm -rf /var/lib/apt/lists/*

    # ....Pip install.............................................................................
    echo "(pip) Install F110-gym dependencies..."

#    # Fix for gym <-> F1TENTH TypeError:
#    # >>> obs, step_reward, done, info = env.reset(np.array([[conf.sx, conf.sy, conf.stheta]]))
#    # >>>     TypeError: reset() takes 1 positional argument but 2 were given
#    # Check pull request #35 "reset() function signature no longer works with gym 0.21.0" at https://github.com/f1tenth/f1tenth_gym/issues/35
#    # Gym version: 0.21.0 -> oct 6 2021; 0.20.0 -> Sep 14, 2021;
#    pip3 install --no-cache-dir 'gym[classic_control,box2d]<0.21.0'

#    # Legacy install step
#    pip3 install --no-cache-dir --upgrade pip \
#    && pip3 install --no-cache-dir  \
#      'gym[classic_control,box2d]==0.19.0'

    # Note: fix openai gym + f110 gym install error
    #   See "ERROR: Failed building wheel for gym" https://github.com/freqtrade/freqtrade/issues/8376#issuecomment-1520844382
    # Note: pip<24.1 is required for backward compatibility
    # (CRITICAL) ToDo: Validate droping the upgrade flag (ref task NMO-768)
    #pip3 install --upgrade \
    pip3 install --no-cache-dir \
        'pip<24.1' \
        'wheel==0.38.4' \
        'setuptools==65.5.1' \
        || exit 1

    # ....F1TENTH install.............................................................................
    if [[ -d /opt/f1tenth_gym ]]; then \
     tree -L 1 -a && echo "/opt/f1tenth_gym already exist!" && exit 1 ; \
    fi

    echo "Install F110-gym..."
    # Notes › f110_gym:
    #   - "editable_mode=compat" is a quickhack for the pip editable mode issue related to setuptools
    #   - f110_gym need to be installed in editable mode because there is curently no MANIFESTE.in file
    #       which prevent some required assets (e.g. maps/) to be included in the install path
    cd /opt
    git clone https://github.com/f1tenth/f1tenth_gym.git
    cd ./f1tenth_gym
    pip3 install -e . --config-settings editable_mode=compat || exit 1


    # Quick hack to fix `ImportError: Can't find framework /System/Library/Frameworks/OpenGL.framework.`
    # See known issues comment https://github.com/f1tenth/f1tenth_gym
    # (CRITICAL) ToDo: Validate droping the upgrade flag (ref task NMO-768)
    #pip3 install --no-cache-dir --upgrade 'pyglet<1.5.11'
#    pip3 install --no-cache-dir 'pyglet<1.5.11'
    pip3 install --no-cache-dir 'pyglet==1.5.20'
    #pip3 install --no-cache-dir --upgrade 'pyglet!=1.5.11'

EOF


FROM main AS test
RUN <<EOF
    echo -e "
----------------------------------------------------------
Container build time environment variables
$(printenv)

----------------------------------------------------------
Log dn_bash_alias.bash state
$(cat /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash)

----------------------------------------------------------
Python version: $(python --version)
Numpy version: $(pip list | grep -i -e "numpy ")

which python:
$(which python)

whereis python:
$(whereis python)

which pip's:
$(which pip pip3)

pip list:
$(pip list)

----------------------------------------------------------
$(pip3 show pycuda)
$(python3 -c "import pycuda; print( pycuda.VERSION_TEXT )")
----------------------------------------------------------
$(pip3 show torch)
$(python3 -c "import torch; print( f'CUDA is available: {torch.cuda.is_available()}' )")
----------------------------------------------------------
$(pip3 show f110_gym)
----------------------------------------------------------
Sanity check...
"
    python /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_gym.py || exit 1
    python /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_pytorch_cpu.py || exit 1
    python3 -c "import f110_gym" || exit 1
    python3 -c "from f110_gym.envs import F110Env" || exit 1
    python3 -c "import gym; env = gym.make('f110_gym:f110-v0', num_agents=1, timestep=0.01, ego_idx=0)" || exit 1
EOF

FROM main AS final

