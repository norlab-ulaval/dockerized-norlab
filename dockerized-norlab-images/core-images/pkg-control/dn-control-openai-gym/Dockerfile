
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS main

COPY --from=context-dn-container-tools ./dn_test/try_gym.py /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/

RUN <<EOF
    # Dependencies
    #   - box2d-py dependency: swig
    #   - gym VideoRecorder dependency: ffmpeg and imageio-ffmpeg
    #   - gym headless server rendering require a virtual X server like xvfb
    #       pyvirtualdisplay is a python wraper for xvfb › https://github.com/ponty/pyvirtualdisplay/tree/3.0
    #       gym-notebook-wrapper is wrapper for running gym and recording movie on Jupyter notebook
    #           ↳ |  require xvfb and python-opengl

    # ....Deb install..............................................................................
    apt-get update

    echo "(deb) Install OpenAI gym dependencies..."
    apt-get install --assume-yes  --no-install-recommends \
            swig \
            mesa-utils \
            ffmpeg

    echo "Install pyvirtualdisplay dependencies and backends on Ubuntu 20.04..."
    apt-get install --assume-yes  --no-install-recommends \
            xvfb \
            xserver-xephyr \
            tigervnc-standalone-server \
            x11-utils \
            gnumeric

#            python3-opengl \

    rm -rf /var/lib/apt/lists/*
    apt-get clean

    # ....Pip install..............................................................................
    pip3 install --no-cache-dir --upgrade pip

    #pip3 install --no-cache-dir --upgrade Cython \
    #echo "Install >>> pip cython $( pip list | grep -i cython )..."

    echo "Install vizualisation tools..."
    pip3 install --no-cache-dir  \
        pyvirtualdisplay \
        imageio-ffmpeg \
        PyOpenGL \
        PyOpenGL_accelerate

    echo "(pip) Install OpenAI gym dependencies..."
    pip3 install --no-cache-dir  \
        gym[classic_control,box2d]  \
        pyglet \
        box2d-py

    echo "Install Jupyter + gym related..."
    pip3 install --no-cache-dir  \
        jupyterlab \
        notebook \
        gym-notebook-wrapper

    # ....Add gym dn_test module...................................................................
    ( \
      echo "alias dn_gym_check='python3 /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_gym.py'"; \
    ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bashrc.bash

EOF


# ToDo: on task end >> mute next bloc ↓↓
FROM main AS test

RUN <<EOF
    echo "Container build time environment variables"
    echo
    printenv
    echo
    cat /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bashrc.bash
    echo

    echo "Sanity check..."
    python /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_gym.py
EOF
