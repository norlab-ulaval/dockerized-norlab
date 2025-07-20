FROM ubuntu:22.04

ARG IS_TEAMCITY_RUN
ENV IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:-false}

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

ENV TZ=Etc/UTC
ENV TERM=${TERM:-"xterm-256color"}

CMD [ "bash" ]

