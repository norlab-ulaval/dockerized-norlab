
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS base-image


# ....install development utilities................................................................................
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        apt-utils \
        lsb-release \
        cmake \
        build-essential \
        curl \
        wget \
        gnupg2 \
        ca-certificates \
        git \
        libusb-dev \
        usbutils \
        vim \
        tree \
        zip gzip tar unzip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# ===Install python scientific stack ===================================================================================
FROM base-image AS scientific-stack

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

# ===install utilities==================================================================================================
#FROM ros-performance-pkg-base-image AS python-dev-tools
FROM scientific-stack AS python-dev-tools

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



# ===Final build step===================================================================================================
FROM python-dev-tools AS final

# ...install development utilities......................................................................................
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        usbutils \
        rsync \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# ....Set DN_PYTHON3_VERSION for powerline..........................................................................
WORKDIR /dockerized-norlab

COPY ./utilities/norlab-shell-script-tools/ ./utilities/norlab-shell-script-tools/

RUN cd ./utilities/norlab-shell-script-tools/src/utility_scripts \
    && source ./which_python_version.bash \
    && echo "PYTHON3_VERSION=${PYTHON3_VERSION}"

COPY ./dockerized-norlab-images/container-tools/ ./dockerized-norlab-images/container-tools/

RUN ( \
  echo ""; \
  echo "# dockerized-norlab aliases (from dependencies img)"; \
  echo "alias dn_info='bash /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash'"; \
  echo "alias dn_python3_check='python3 /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_pytorch.py'"; \
  echo "alias dn_gym_check='python3 /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_gym.py'"; \
  echo ""; \
) >> ~/.bashrc

RUN bash /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash

CMD [ "bash" ]



