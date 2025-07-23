
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS main

COPY requirements_deep_rl.txt /requirements_deep_rl.txt

# Doc › pip install flag: https://pip.pypa.io/en/stable/cli/pip_install/#options

RUN <<EOF
    pip3 install --no-cache-dir -r /requirements_deep_rl.txt || exit 1
    rm -rf /requirements_deep_rl.txt

    if [[ "$( source /etc/lsb-release && echo ${DISTRIB_CODENAME})" != "focal"  ]]; then
        # ....torch-RL.............................................................................
        # Ref:
        #  - https://docs.pytorch.org/rl/stable/index.html#installation
        #  - https://github.com/pytorch-labs/tensordict
        pip install torchrl
    else
      echo "Skipping TorchRL install as TensorDict is not available on Ubuntu Focal"
    fi

EOF

FROM main AS test

RUN <<EOF
    echo "Sanity check"
    python -c "import torch" \
      && python -c "from ray import train, tune" \
      && python -c "from ray.tune.search.hyperopt import HyperOptSearch" \
      || exit 1

    if [[ "$( source /etc/lsb-release && echo ${DISTRIB_CODENAME})" != "focal"  ]]; then
      python -c "import torchrl" \
        && python -c "from tensordict.tensordict import TensorDict" \
        || exit 1
    fi
EOF

FROM main AS final

