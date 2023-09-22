
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS base-image

# ====Begin=============================================================================================
COPY requirements_deep_rl.txt /requirements_deep_rl.txt

# Doc › pip install flag: https://pip.pypa.io/en/stable/cli/pip_install/#options
RUN pip3 install --no-cache-dir --verbose -r /requirements_deep_rl.txt  \
    && rm -rf /requirements_deep_rl.txt


# ====End===============================================================================================
CMD [ "bash" ]

