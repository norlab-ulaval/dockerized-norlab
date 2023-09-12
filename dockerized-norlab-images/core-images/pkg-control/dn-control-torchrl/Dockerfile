
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS base-image

# ====Begin=============================================================================================

# ....TorchRL......................................................................................................
# Note: ON HOLD › Both TorchRL and TensorDict are in beta release -> instalation is problematic and bugy on arch64.
# Either
#   - it build and pass the sanity check in docker build but break on docker run `import torchrl`
#   - or it build and work in docker run `import torchrl` but break `torch.cuda.is_available()`

# QuickFix:
#   - Fix warning: `Failed to load image Python extension: {e}` cause by torch and torchvision incompatible version,
#     (see torch <-> torchvision compatible pair: https://github.com/pytorch/vision#installation)


RUN mkdir /pytorch-labs && cd /pytorch-labs  \
    && git clone https://github.com/pytorch-labs/tensordict.git  \
    && cd /pytorch-labs/tensordict && pip3 install -e .

RUN pip3 install --no-cache-dir --upgrade torchvision

RUN mkdir /torchrl && cd /torchrl  \
    && git clone https://github.com/pytorch/rl  \
    && cd ./rl && pip3 install -e .

#RUN pip3 install --no-cache-dir \
#  git+https://github.com/pytorch-labs/tensordict@v0.1.0 \
#  git+https://github.com/pytorch/rl@v0.1.0

#RUN pip3 install --no-cache-dir \
#    git+https://github.com/pytorch-labs/tensordict \
#    git+https://github.com/pytorch/rl
#
#RUN python -c "import torch" \
#    && python -c "import torchvision" \
#    && python -c "import tensordict" \
#    && python -c "import torchrl"


# ====End===============================================================================================
CMD [ "bash" ]

