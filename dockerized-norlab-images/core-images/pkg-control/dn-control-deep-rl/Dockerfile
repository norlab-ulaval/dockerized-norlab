
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS main

COPY requirements_deep_rl.txt /requirements_deep_rl.txt

# Doc › pip install flag: https://pip.pypa.io/en/stable/cli/pip_install/#options

RUN <<EOF
    if [[ "${DISTRIB_CODENAME}" == "focal"  ]]; then
        # ....pyopenssl/cryptography fix...........................................................
        # Fix AttributeError: module 'lib' has no attribute 'X509_V_FLAG_NOTIFY_POLICY' on Ubuntu
        # focal (Python 3.8, linux/arm64). The system python3-openssl package is stale (< 23.2.0)
        # and still references X509StoreFlags.NOTIFY_POLICY, which was removed in cryptography
        # >= 42.0.0. Upgrading via pip ensures /usr/local/lib/python3.8/dist-packages/OpenSSL
        # (pyopenssl >= 23.2.0) is used instead of the stale system package.
        # Ref: https://github.com/pyca/pyopenssl/pull/1213
        pip3 install --no-cache-dir \
          pyopenssl \
          cryptography
    fi


    pip3 install --no-cache-dir -r /requirements_deep_rl.txt || exit 1
    rm -rf /requirements_deep_rl.txt

    source /etc/lsb-release
    if [[ "${DISTRIB_CODENAME}" != "focal"  ]]; then
        # ....torch-RL.............................................................................
        # Ref:
        #  - https://docs.pytorch.org/rl/stable/index.html#installation
        #  - https://github.com/pytorch-labs/tensordict
        pip install torchrl
    elif [[ "${DISTRIB_CODENAME}" == "focal"  ]]; then

        # python 3.8 compatible tensordict repo tags:
        #   - 0.0.1
        #   - v0.1.2
        #   - v0.4.0 <- is the last compatible one (considering external dependencies)
        #   - ! v0.5.0
        # Ref
        # - https://github.com/pytorch/tensordict/tree/v0.4.0
        #   https://github.com/pytorch/tensordict/blob/v0.4.0/setup.py
        # - https://github.com/pytorch/rl/tree/v0.4.0
        #   https://github.com/pytorch/rl/blob/v0.4.0/setup.py
        PYTHON38_COMPAT_VS=v0.4.0
        echo "Installing TorchRL and TensorDict version ${PYTHON38_COMPAT_VS} -> best compatible version for python 3.8"

        # Note: don't clone to "${DN_DEV_WORKSPACE}/src" (i.e., /ros2_src/src) as it break colcon build.
        cd "/opt"

        if ! pip show tensordict -qq; then
            git clone --branch "${PYTHON38_COMPAT_VS}" https://github.com/pytorch/tensordict
            cd tensordict
            python -m pip install .
        fi

        cd ..
        git clone --branch "${PYTHON38_COMPAT_VS}" https://github.com/pytorch/rl
        cd rl
        python -m pip install .
    fi

EOF

FROM main AS test

RUN <<EOF
    echo "Sanity check"

    python -c "import torch" \
      && python -c "from ray import train, tune" \
      && python -c "from ray.tune.search.hyperopt import HyperOptSearch" \
      || exit 1

    python -c "import torchrl" \
      && python -c "from tensordict.tensordict import TensorDict" \
      || exit 1
EOF

FROM main AS final

