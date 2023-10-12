
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS norlab-teamcity-server-setup


SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive


RUN ${DN_PROJECT_GIT_NAME:?'Build argument needs to be set and non-empty.'} \
    && echo ${DN_PROJECT_GIT_DOMAIN:?'Build argument needs to be set and non-empty.'}


WORKDIR /

# ===install teamcity & CI/CD utilities==================================================================

RUN pip3 install --no-cache-dir --verbose --upgrade pip
RUN pip3 install --no-cache-dir --verbose --upgrade pytest
RUN pip3 install --no-cache-dir --verbose teamcity-messages

RUN ( \
  echo "Configure Matplotlib to use a non-interactive backends for TeamCity run"; \
  echo "backend : Agg" >> /etc/matplotlibrc; \
)


# ... Finish container setup ............................................................
WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
CMD [ "bash" ]
