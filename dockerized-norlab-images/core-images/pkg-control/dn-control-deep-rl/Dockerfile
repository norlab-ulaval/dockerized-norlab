
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS main
WORKDIR /opt

COPY requirements_deep_rl.txt /requirements_deep_rl.txt

# Doc › pip install flag: https://pip.pypa.io/en/stable/cli/pip_install/#options
RUN pip3 install --no-cache-dir -r /requirements_deep_rl.txt  \
    && rm -rf /requirements_deep_rl.txt

FROM main AS test

RUN echo "Sanity check" \
    && python -c "import torch" \
    && python -c "import pytorch_mppi" \
    && python -c "from pytorch_mppi import MPPI" \
    && python -c "from ray import train, tune" \
    && python -c "from ray.tune.search.hyperopt import HyperOptSearch" \
    && python -c "from pytorch_mppi import autotune" \
    && python -c "from stable_baselines3 import SAC"

FROM main AS final
WORKDIR ${DN_DEV_WORKSPACE}
CMD [ "bash" ]

