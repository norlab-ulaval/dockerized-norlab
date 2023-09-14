
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS base-image

# ====Begin=============================================================================================

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


##RUN pip3 uninstall -y opencv-python  \
##    && pip3 install opencv-python-headless

## ....F1TENTH docker file install....................................................................
#RUN apt-get update --fix-missing && \
#    apt-get install -y \
#                    python3-dev \
#                    python3-pip \
#                    build-essential \
#                    libgl1-mesa-dev \
#                    mesa-utils \
#                    libglu1-mesa-dev \
#                    fontconfig \
#                    libfreetype6-dev
#
#RUN pip3 install --upgrade pip \
#  && pip3 install PyOpenGL \
#                 PyOpenGL_accelerate
## ...................................................................................................

### (☕MINOR) ToDo: test >> next bloc ↓↓
#RUN apt-get update --fix-missing \
#    && apt-get upgrade --assume-yes
#
### (☕MINOR) ToDo: test >> next bloc ↓↓
##RUN pip3 uninstall --verbose setuptools \
##    && pip3 install --no-cache-dir --verbose --upgrade \
##           setuptools
#RUN pip3 install --no-cache-dir --verbose --upgrade --force-reinstall \
#           setuptools

# ToDo: assessment python version installed in dustynv pytorch image
RUN bash /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash

# ...................................................................................................
RUN pip3 install --no-cache-dir --upgrade pip \
    && pip3 install --no-cache-dir --verbose \
      'gym[classic_control,box2d]==0.19.0'

WORKDIR /
RUN git clone https://github.com/f1tenth/f1tenth_gym.git  \
    && cd /f1tenth_gym  \
    && pip3 install -e .
#    && pip3 install -e ".[dev]"


# Quick hack to fix `ImportError: Can't find framework /System/Library/Frameworks/OpenGL.framework.`
# See known issues comment https://github.com/f1tenth/f1tenth_gym
#RUN pip3 install pyglet==1.5.11
RUN pip3 install --no-cache-dir --upgrade  \
    'pyglet<1.5.11' \
    pyopengl

# ====End===============================================================================================
CMD [ "bash" ]

