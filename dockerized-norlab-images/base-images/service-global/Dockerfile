
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack
ARG BASE_IMAGE_TAG=r35.2.1
FROM ${BASE_IMAGE}:${BASE_IMAGE_TAG}

LABEL org.opencontainers.image.authors="luc.coupal.1@ulaval.ca"

ARG IS_TEAMCITY_RUN
ENV IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:-false}

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_FRONTEND=${DEBIAN_FRONTEND}

ENV TZ=Etc/UTC
ENV TERM=${TERM:-"xterm-256color"}


# ====End==========================================================================================================
CMD [ "bash" ]

