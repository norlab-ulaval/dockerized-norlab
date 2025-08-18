
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS main

COPY --from=context-dn-container-tools ./dn_test/try_gym.py /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/

RUN <<EOF
    # ....Dependencies notes.......................................................................
    #   - box2d-py dependency: swig
    #   - gym VideoRecorder dependency: ffmpeg and imageio-ffmpeg
    #   - gym headless server rendering require a virtual X server like xvfb
    #       pyvirtualdisplay is a python wraper for xvfb › https://github.com/ponty/pyvirtualdisplay/tree/3.0
    #       gym-notebook-wrapper is wrapper for running gym and recording movie on Jupyter notebook
    #           ↳ |  require xvfb and python-opengl
    # .............................................................................................

    # ....Deb install..............................................................................
    apt-get update

    echo "(deb) Install OpenAI gym dependencies..."
    apt-get install --assume-yes  --no-install-recommends \
            swig \
            mesa-utils \
            ffmpeg \
        || exit 1

    echo "Install pyvirtualdisplay dependencies and backends on Ubuntu 20.04..."
    # Note:
    #   Do NOT install opengl using deb i.e., 'python3-opengl' ⚠️
    #   Use pip install PyOpenGL instead.
    apt-get install --assume-yes  --no-install-recommends \
            xvfb \
            xserver-xephyr \
            tigervnc-standalone-server \
            x11-utils \
            gnumeric \
        || exit 1

    apt-get autoremove --assume-yes
    apt-get clean
    rm -rf /var/lib/apt/lists/*

    # ....Pip install..............................................................................
    pip3 install --no-cache-dir --upgrade pip

    #pip3 install --no-cache-dir Cython \
    #echo "Install >>> pip cython $( pip list | grep -i cython )..."

    echo "Install vizualisation tools..."
    pip3 install --no-cache-dir  \
        pyvirtualdisplay \
        imageio-ffmpeg \
        PyOpenGL

        #PyOpenGL_accelerate # Might only provides optional performance optimizations

    echo "(pip) Install OpenAI gym dependencies..."
    # Note: OpenAI gym is not compatible with numpy 2.0, so pin to earlier version ⚠️
    pip3 install --no-cache-dir \
        'numpy<2.0.0' \
        || exit 1

    pip3 install --no-cache-dir  \
        gym[classic_control,box2d]  \
        pyglet \
        box2d-py \
      || exit 1

    echo "Install Jupyter + gym related..."
    pip3 install --no-cache-dir  \
        jupyterlab \
        notebook \
        gym-notebook-wrapper

    # ....Add gym dn_test module...................................................................
    (
      echo "alias dn-gym-check='python3 /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_gym.py'"
    ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash


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
Sanity check...
"
    python /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_gym.py || exit 1
    python /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_pytorch_cpu.py || exit 1
EOF
