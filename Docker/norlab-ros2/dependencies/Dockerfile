
# /// norlab-ros2/dependencies /////////////////////////////////////////////////////////////////////////////////////////
# This container is the base image for all other norlab-ros images: dev, deploy, ...
#
# References:
# - NVIDIA Container Runtime on Jetson: https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson
# - dusty-nv/jetson-containers: https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.noetic
# - ROS noetic installation from source: http://wiki.ros.org/noetic/Installation/Source

# Base image: nvidia linux 4 tegra (L4T) nvidia docker container official image
#   l4t-base: https://ngc.nvidia.com/catalog/containers/nvidia:l4t-base
#   l4t-ros-noetic-pytorch: https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic

#ARG BASE_IMAGE=nvidiajetson/l4t-ros-noetic-pytorch:r32.5
#ARG BASE_IMAGE=norlabsnow/norlab-ros2-humble:x86-ubuntu20.04
ARG DN_BASE_IMG_TAG=humble-pytorch-arm64-l4t-r35.2.1
ARG BASE_IMAGE=norlabsnow/norlab-ros
FROM ${BASE_IMAGE}:${DN_BASE_IMG_TAG} AS ros-pytorch-base-image


# ....setup environment............................................................................................


WORKDIR "/"

## (NICE TO HAVE) todo:investigate?? ( NLSAR-226 ) >> ln: failed to create symbolic link '/etc/localtime': File exists
## Setup timezone
##   Source: OSRF docker_images
##       https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/bionic/ros-core/Dockerfile
#RUN echo 'Etc/UTC' > /etc/timezone && \
#    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
#    apt-get update && \
#    apt-get install -q -y --no-install-recommends tzdata && \
#    rm -rf /var/lib/apt/lists/*




# ===PyTorch============================================================================================================
#FROM ros-python3-base-image AS ros-pytorch-base-image

ENV CUDA_HOME="/usr/local/cuda"
ENV LLVM_CONFIG="/usr/bin/llvm-config-9"

ARG MAKEFLAGS=-j$(nproc)



# (CRITICAL) ToDo: validate
COPY dn_which_python_version.bash /
RUN bash dn_which_python_version.bash
ENV DS_PYTHON3_VERSION="${PYTHON3_VERSION}"
#ARG PYTHON3_VERSION=3.8


ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

RUN apt-get update \
    && apt-get install -y software-properties-common \
    && ldconfig \
    && apt-get install -y --no-install-recommends \
            libopenblas-base \
            libopenblas-dev \
            libomp-dev \
            libopenmpi-dev \
            python3-pip \
            python3-dev \
            openmpi-bin \
            openmpi-common \
            gfortran \
    && rm -rf /var/lib/apt/lists/*
#
## Note: libopenblas-base, libopenmpi-dev, python3-pip, Cython, numpy are all requirement for PyTorch
##       build for JetPack (L4T). https://elinux.org/Jetson_Zoo#PyTorch_.28Caffe2.29
#
RUN pip3 install --no-cache-dir --verbose --upgrade pip  \
    && pip3 install --no-cache-dir --verbose  \
        setuptools \
        wheel \
        Cython \
        numpy



ENV PYTORCH_VERSION = 2.0.0
ARG TORCHVISION_VERSION=0.14.1
## . . PyTorch (for JetPack) . . . . . . . . . . . . . . . . . . . . . .
##  https://elinux.org/Jetson_Zoo#PyTorch_.28Caffe2.29
##
## 💎 Jetpack 5.1 (PyTorch v2.0) for L4T R35.2.1
ARG PYTORCH_L4T_WHL_URL=https://nvidia.box.com/shared/static/rehpfc4dwsxuhpv4jgqv8u6rzpgb64bq.whl
ARG PYTORCH_L4T_WHL=torch-2.0.0a0+ec3941ad.nv23.2-cp38-cp38-linux_aarch64.whl
## Matching torchvision version
#ARG TORCHVISION_VERSION="v0.14.1"
## Note: torch-2.0.0-cp38-cp38-manylinux2014_aarch64.whl

## PyTorch (official)
#ARG PYTORCH_OFFICIAL_WHL_URL=https://download.pytorch.org/whl/torch_stable.html
#
### 💎 . . Pytorch for M1 chips . . . . . . . . . . . . . . . . . . . . . .
### PyTorch v1.9.0 https://download.pytorch.org/whl/cpu/torch-1.9.0-cp39-none-macosx_11_0_arm64.whl cpu/torch-1.9.0-cp39-none-macosx_11_0_arm64.whl
### PyTorch v1.9.1 https://download.pytorch.org/whl/cpu/torch-1.9.1-cp39-none-macosx_11_0_arm64.whl cpu/torch-1.9.1-cp39-none-macosx_11_0_arm64.whl
### Pytorch v2.0.0 https://download.pytorch.org/whl/cpu/torch-2.0.0-cp39-none-macosx_11_0_arm64.whl cpu/torch-2.0.0-cp39-none-macosx_11_0_arm64.whl
##ARG PYTORCH_URL_M1=https://download.pytorch.org/whl/cpu/torch-2.0.0-cp38-none-macosx_11_0_arm64.whl
##ARG PYTORCH_WHL_M1=torch-2.0.0-cp38-none-macosx_11_0_arm64.whl

#
## 💎 . . Pytorch for x86 . . . . . . . . . . . . . . . . . . . . . . . . .
## https://pytorch.org/get-started/locally/
## cu111 ==> cuda version
##ARG PYTORCH_VERSION_X86="1.9.0+cu111"
#ARG PYTORCH_WHL_X86=torch-2.0.0+cpu-cp38-cp38-linux_x86_64.whl
#
## (CRITICAL) todo:fixme!! (ref task NLSAR-230)
## Conditional build stage base on architecture version (arm64-l4t and x86)

# Torchvision related build requirement
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        build-essential \
        ninja-build \
        libjpeg-dev \
        zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

## Note: wget -O save the file under a diferent name
RUN /bin/bash -c "if [[ ${DN_IMAGE_ARCHITECTURE} == 'arm64-l4t' ]]; then \
        wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_L4T_WHL_URL} -O ${PYTORCH_L4T_WHL} && \
        pip3 install --no-cache-dir --verbose ${PYTORCH_L4T_WHL} --verbose && \
        rm ${PYTORCH_L4T_WHL}; \
        git clone --branch v${TORCHVISION_VERSION} --recursive --depth=1 https://github.com/pytorch/vision torchvision && \
        cd torchvision && \
        git checkout v${TORCHVISION_VERSION} && \
        python3 setup.py install && \
        cd ../ && \
        rm -rf torchvision  \
    elif [[ ${DN_IMAGE_ARCHITECTURE} == 'arm64-Darwin' ]]; then \
      pip3 install --no-cache-dir --verbose torch==${PYTORCH_VERSION} torchvision==${TORCHVISION_VERSION} \
      rm ${PYTORCH_WHL_M1}; \
    elif [[ ${DN_IMAGE_ARCHITECTURE} == 'x86' ]]; then \
      pip3 install --no-cache-dir --verbose torch==${PYTORCH_VERSION} torchvision==${TORCHVISION_VERSION} \
    else \
      echo 'Architecture ${DN_IMAGE_ARCHITECTURE} is not curently suported'; \
      exit 1; \
    fi"





#RUN /bin/bash -c "if [[ ${DN_IMAGE_ARCHITECTURE} == 'arm64-l4t' ]]; then \
#        wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_L4T_WHL_URL} -O ${PYTORCH_L4T_WHL} && \
#        pip3 install --no-cache-dir --verbose ${PYTORCH_L4T_WHL} --verbose && \
#        rm ${PYTORCH_L4T_WHL}; \
#    elif [[ ${DN_IMAGE_ARCHITECTURE} == 'arm64-Darwin' ]]; then \
#      pip3 install --no-cache-dir --verbose torch==1.9.0 -f ${PYTORCH_OFFICIAL_WHL_URL}; \
#    elif [[ ${DN_IMAGE_ARCHITECTURE} == 'x86' ]]; then \
#      pip3 install --no-cache-dir --verbose torch==${PYTORCH_VERSION_X86} -f ${PYTORCH_OFFICIAL_WHL_URL}; \
#    else \
#      echo 'Architecture ${DN_IMAGE_ARCHITECTURE} is not curently suported'; \
#      exit 1; \
#    fi"




# ===Install python scientific stack ===================================================================================
FROM ros-pytorch-base-image AS ros-scientific-stack-base-image

# install aditional python package
RUN pip3 install --no-cache-dir --verbose \
    scipy \
    scikit-learn \
    pandas \
    numba \
    pycuda

RUN pip3 install --no-cache-dir --ignore-installed pybind11

# ...Configure Matplotlib to use a non-interactive backends for TeamCity run............................................
# Procedure to fix NLSAR-343 ﹅→ `ModuleNotFoundError` when using pyplot & `couldn't connect to display`
# Solution: install python3-tk and configure matplotlib to use a non-interactive backend in TeamCity
# Ref:
#   - https://matplotlib.org/2.0.2/faq/usage_faq.html#what-is-a-backend
#   - https://matplotlib.org/stable/tutorials/introductory/customizing.html?highlight=matplotlibrc#the-matplotlibrc-file
#
## Option 1: Set the MPLBACKEND environment variable at container level in `matplotlibrc`
#RUN echo "backend : Agg" >> /etc/matplotlibrc
#
## Option 2: Set the MPLBACKEND environment variable manualy in the TeamCity build step
#export MPLBACKEND="Agg"
#python3 -m pytest tests/

# Hack to install matplotlib on arm64 and it's required module tkinter
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
      python3-matplotlib \
      python3-tk \
    && rm -rf /var/lib/apt/lists/*

## . . install OpenCV (with CUDA). . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
##  Source: https://github.com/dusty-nv/jetson-containers/blob/bd40fdfc41b912b93de1c33ec91cd0534cf2fcc6/Dockerfile.ml#L139
#ARG OPENCV_URL=https://nvidia.box.com/shared/static/5v89u6g5rb62fpz4lh0rz531ajo2t5ef.gz
#ARG OPENCV_DEB=OpenCV-4.5.0-aarch64.tar.gz
#
#COPY scripts/opencv_install.sh /tmp/opencv_install.sh
#RUN cd /tmp && ./opencv_install.sh ${OPENCV_URL} ${OPENCV_DEB}

# ===Install performance optimization package===========================================================================
#FROM ros-scientific-stack-base-image AS ros-performance-pkg-base-image

# . . Numba install. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# https://numba.pydata.org

## LLVM v3.9 <== (Python3.6 + JetPack 4.3/4.4)
## Numba v0.31 <== (Python3.6 + JetPack 4.3/4.4)
##ENV LVL_VERSION=3.9
##ENV LVLLITE_VERSION=0.16.0
#ENV LVL_VERSION=4.0
#ENV LVLLITE_VERSION=0.20.0
#
## Inspired by https://gitee.com/Cheng_Loon/jetson-packages-family-good
#RUN apt-get update \
#    && apt-get install --assume-yes --no-install-recommends \
#        llvm-${LVL_VERSION} \
#        llvm-${LVL_VERSION}-dev \
#    && export LLVM_CONFIG=/usr/lib/llvm-${LVL_VERSION}/bin/llvm-config \
#    && cd ${HOME} \
#    && wget https://github.com/numba/llvmlite/archive/v${LVLLITE_VERSION}.zip \
#    && unzip v${LVLLITE_VERSION}.zip \
#    && cd llvmlite-${LVLLITE_VERSION} \
#    && chmod 777 -R /usr/local/lib/python3.6/dist-packages/ \
#    && python3 setup.py install
#
#RUN apt-get update \
#    && apt-get install --assume-yes --no-install-recommends \
#    python3-numba

#RUN pip3 install --no-cache-dir --verbose numba



# ===install utilities==================================================================================================
#FROM ros-performance-pkg-base-image AS python-dev-tools
FROM ros-scientific-stack-base-image AS python-dev-tools

# ...Install simulator..................................................................................................
# Dependencies
#   box2d-py dependency: swig
#   Gym headless server rendering require a virtual X server like xvfb
#       pyvirtualdisplay is a python wraper for xvfb › https://github.com/ponty/pyvirtualdisplay/tree/3.0
#       gym-notebook-wrapper is wrapper for running gym and recording movie on Jupyter notebook
#           ↳ |  require xvfb and python-opengl
#   gym VideoRecorder dependency: ffmpeg and imageio-ffmpeg
RUN apt-get update \
#    && apt-get install --assume-yes --no-install-recommends \
    && apt-get install --assume-yes \
        swig \
        mesa-utils \
        ffmpeg \
        # . .pyvirtualdisplay dependencies and backends on Ubuntu 20.04 . . .
        xvfb \
        python-opengl \
        xserver-xephyr \
        tigervnc-standalone-server \
        x11-utils \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --verbose --upgrade pip  \
    && pip3 install --no-cache-dir --verbose  \
        # . . Vizualisation tools. . . . . . . . . . . . . . . . . . . . . .
        pyvirtualdisplay \
        imageio-ffmpeg  \
        PyOpenGL \
        PyOpenGL_accelerate \
        # . . Gym depencies . . . . . . . . . . . . . . . . . . . . . . . . . .
        gym  \
        pyglet \
        box2d-py \
        # . . Jupyter + gym related . . . . . . . . . . . . . . . . . . . . . .
        jupyterlab \
        notebook \
        gym-notebook-wrapper \
        # . . install python general development package . . . . . . . . . . . .
        pyyaml \
        termcolor \
        decorator \
        black[d] \
        # . . Type check. . . . . . . . . . . . . . . . . . . . . . . . . . . .
        pytype \
        mypy \
        # . . Testing tools . . . . . . . . . . . . . . . . . . . . . . . . . .
        pytest \
        pytest-benchmark \
        pytest-mock \
        python-dotenv \
        pytest-env \
        mock

        ## (!) pytest-cov › "The pytest-cov package, due to technical restrictions, breaks PyCharm's debugger." \
        ## see https://www.jetbrains.com/help/pycharm/2023.1/run-debug-configuration-py-test.html

## Packages for older python version
#RUN pip3 install --no-cache-dir --verbose  \
#            dataclasses

#RUN pip3 install --no-cache-dir --verbose -U mock


# ===Final build step===================================================================================================
FROM python-dev-tools AS final

# ...install development utilities......................................................................................
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        usbutils \
        rsync \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /
COPY ./tools/dn_info.bash /
COPY ./tools/dn_test/try_pytorch.py dn_test/
COPY ./tools/dn_test/try_gym.py dn_test/

RUN ( \
  echo ""; \
  echo "# dockerized-norlab aliases (from dependencies img)"; \
  echo "alias dn_info='bash /dn_info.bash'"; \
  echo "alias dn_python3_check='python3 /dn_test/try_pytorch.py'"; \
  echo "alias dn_gym_check='python3 /dn_test/try_gym.py'"; \
  echo ""; \
) >> ~/.bashrc


CMD [ "bash" ]
# ///////////////////////////////////////////////////////////////////////////////////////// norlab-ros2/dependencies ///



