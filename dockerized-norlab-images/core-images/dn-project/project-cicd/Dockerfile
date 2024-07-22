
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS norlab-teamcity-cicd-setup

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

RUN echo "Pre-condition checks" && { \
        test $(whoami) == "root" &&  \
        test -n ${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'} && \
        test -d /home/${DN_PROJECT_USER} && \
        test -n ${DN_PROJECT_GIT_NAME:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_GIT_DOMAIN:?'Env variable need to be set and non-empty.'} ; \
    } || exit 1

# ===install teamcity & CI/CD utilities============================================================

RUN pip3 install --no-cache-dir  --upgrade pip
RUN pip3 install --no-cache-dir teamcity-messages

RUN ( \
  echo "Configure Matplotlib to use a non-interactive backends for TeamCity run"; \
  echo "backend : Agg" >> /etc/matplotlibrc; \
)

# ====Execute tests================================================================================
FROM norlab-teamcity-cicd-setup AS execute-tests
USER ${DN_PROJECT_USER}
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ENV PYTEST_CI_RUN=true

WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}

# ====Project custom steps===========================================================================
# ToDo: add custom step here


# ... Finish container setup ............................................................
WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
CMD [ "bash" ]
