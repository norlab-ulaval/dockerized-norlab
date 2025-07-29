
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS main

COPY --from=context-dn-container-tools ./dn_test/try_pytorch.py /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/
COPY --from=context-dn-container-tools ./dn_test/try_pytorch_cpu.py /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/

COPY requirements_deep_learning.txt /requirements_deep_learning.txt

# Doc › pip install flag: https://pip.pypa.io/en/stable/cli/pip_install/#options

RUN <<EOF
    pip3 install --no-cache-dir -r /requirements_deep_learning.txt || exit 1
    rm -rf /requirements_deep_learning.txt

    {
      echo "alias dn_pytorch_cuda_check='python3 /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_pytorch.py'";
      echo "alias dn_pytorch_cpu_check='python3 /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_pytorch_cpu.py'";
    } >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash

    # ....TensorDict distro conditional install....................................................
    # Note: TensorDict and TorchRL was in pre-release stage during python 3.8
    source /etc/lsb-release
    if [[ "${DISTRIB_CODENAME}" != "focal"  ]]; then
        # Ref: https://github.com/pytorch-labs/tensordict
        pip install tensordict
    elif [[ "${DISTRIB_CODENAME}" == "focal"  ]]; then

        # python 3.8 compatible tensordict repo tags:
        #   - 0.0.1
        #   - v0.1.2
        #   - v0.4.0 <- is the last compatible one (considering external dependencies)
        #   - ! v0.5.0
        # Ref
        # - https://github.com/pytorch/tensordict/tree/v0.4.0
        #   https://github.com/pytorch/tensordict/blob/v0.4.0/setup.py
        PYTHON38_COMPAT_VS=v0.4.0
        echo "Installing TensorDict version ${PYTHON38_COMPAT_VS} -> best compatible version for python 3.8"

        # Note: don't clone to "${DN_DEV_WORKSPACE}/src" (i.e., /ros2_src/src) as it break colcon build.
        cd "/opt"

        git clone --branch "${PYTHON38_COMPAT_VS}" https://github.com/pytorch/tensordict
        cd tensordict
        python -m pip install .
    fi

EOF

FROM main AS test

RUN <<EOF
    echo "Sanity check"

    python -c "import numpy" \
      && python -c "import Cython" \
      && python -c "import torch" \
      && python -c "import torchvision" \
      && python -c "from ray import train, tune" \
      && python -c "from ray.tune.search.hyperopt import HyperOptSearch" \
      || exit 1

    python /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_pytorch_cpu.py || exit 1

    python -c "from tensordict.tensordict import TensorDict" || exit 1
EOF

FROM main AS final

