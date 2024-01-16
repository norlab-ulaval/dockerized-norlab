
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS base-image

WORKDIR /dockerized-norlab
COPY --from=context-dn-container-tools ./dn_test/try_gym.py ./dockerized-norlab-images/container-tools/dn_test/

# ...Install simulator..............................................................................
WORKDIR /opt

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
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir --upgrade pip  \
    && pip3 install --no-cache-dir  \
        # . . Vizualisation tools. . . . . . . . . . . . . . . . . . . . . .
        pyvirtualdisplay \
        imageio-ffmpeg  \
        PyOpenGL \
        PyOpenGL_accelerate \
        # . . Gym depencies . . . . . . . . . . . . . . . . . . . . . . . . . .
        gym[classic_control,box2d]  \
        pyglet \
        box2d-py \
        # . . Jupyter + gym related . . . . . . . . . . . . . . . . . . . . . .
        jupyterlab \
        notebook \
        gym-notebook-wrapper \
    && echo \
    && echo "Sanity check" \
    && python /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_gym.py \
    && echo \
    && ( \
        echo "alias dn_gym_check='python3 /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_gym.py'"; \
        ) >> ~/.bashrc


CMD [ "bash" ]
