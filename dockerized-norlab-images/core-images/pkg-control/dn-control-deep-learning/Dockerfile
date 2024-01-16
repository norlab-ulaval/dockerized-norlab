
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS base-image

WORKDIR /dockerized-norlab

COPY --from=context-dn-container-tools ./dn_test/try_pytorch.py ./dockerized-norlab-images/container-tools/dn_test/
COPY --from=context-dn-container-tools ./dn_test/try_pytorch_cpu.py ./dockerized-norlab-images/container-tools/dn_test/

WORKDIR /opt
COPY requirements_deep_learning.txt /requirements_deep_learning.txt

# Doc › pip install flag: https://pip.pypa.io/en/stable/cli/pip_install/#options
RUN pip3 install --no-cache-dir -r /requirements_deep_learning.txt  \
    && rm -rf /requirements_deep_learning.txt \
    && echo \
    && echo "Sanity check" \
    && python -c "import numpy" \
    && python -c "import Cython" \
    && python -c "import torch" \
    && python -c "import torchvision" \
    && python /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_pytorch_cpu.py \
    && echo \
    && ( \
        echo "alias dn_python3_check='python3 /dockerized-norlab/dockerized-norlab-images/container-tools/dn_test/try_pytorch.py'"; \
       ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash

CMD [ "bash" ]

