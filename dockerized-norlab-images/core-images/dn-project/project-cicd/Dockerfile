
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS norlab-teamcity-cicd-setup

SHELL ["/usr/local/bin/bash-dn-buildtime", "-c"]


RUN echo "Pre-condition checks..." && { \
        test $(whoami) == "root" &&  \
        test -n ${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'} && \
        test -d /home/${DN_PROJECT_USER} && \
        test -n ${DN_PROJECT_GIT_NAME:?'Env variable need to be set and non-empty.'} && \
        test -n ${DN_PROJECT_GIT_DOMAIN:?'Env variable need to be set and non-empty.'} ; \
    } || exit 1

# ===install teamcity & CI/CD utilities============================================================

RUN <<EOF
    pip3 install --no-cache-dir  --upgrade pip
    pip3 install --no-cache-dir teamcity-messages

    if [[ -f "/etc/matplotlibrc" ]]; then
        if grep -q 'backend : Agg' "/etc/matplotlibrc" 2>/dev/null; then
            echo "Configure Matplotlib to use a non-interactive backends for TeamCity run"
            echo "backend : Agg" >> /etc/matplotlibrc
        fi
    fi
EOF


# ====Execute tests================================================================================
FROM norlab-teamcity-cicd-setup AS execute-tests
USER ${DN_PROJECT_USER}
SHELL ["/usr/local/bin/bash-dn-buildtime", "-c"]

ENV PYTEST_CI_RUN=true

WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}

# ....Project custom steps.........................................................................
# ToDo: add custom step here


# ====Teardown=====================================================================================
RUN echo "All test have passed!"
CMD [ "bash" ]
