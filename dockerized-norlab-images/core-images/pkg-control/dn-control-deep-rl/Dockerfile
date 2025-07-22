
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS main

COPY requirements_deep_rl.txt /requirements_deep_rl.txt

# Doc › pip install flag: https://pip.pypa.io/en/stable/cli/pip_install/#options

RUN <<EOF
    pip3 install --no-cache-dir -r /requirements_deep_rl.txt || exit 1
    rm -rf /requirements_deep_rl.txt
EOF

FROM main AS test

RUN echo "Sanity check" \
    && python -c "import torch" \
    && python -c "import torchrl" \
    && python -c "from tensordict.tensordict import TensorDict" \
    && python -c "from ray import train, tune" \
    && python -c "from ray.tune.search.hyperopt import HyperOptSearch"

FROM main AS final

