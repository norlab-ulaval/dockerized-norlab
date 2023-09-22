
ARG BASE_IMAGE
ARG DN_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${DN_IMAGE_TAG:?err} AS pytorch-base-image


# ===PyTorch=============================================================================================

ENV CUDA_HOME="/usr/local/cuda"
ENV LLVM_CONFIG="/usr/bin/llvm-config-9"

ARG MAKEFLAGS=-j$(nproc)

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




# ===Install python scientific stack ====================================================================
FROM pytorch-base-image AS ros-scientific-stack-base-image

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


CMD [ "bash" ]



