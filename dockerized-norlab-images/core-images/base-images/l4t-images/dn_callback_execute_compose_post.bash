#!/bin/bash

# ===============================================================================================
# Post docker command execution callback
#
# Usage:
#   $ source dn_callback_execute_compose_post.bash && dn::callback_execute_compose_post
#
# Globals:
#   Read DOCKER_COMPOSE_CMD_ARGS
#   Read DN_HUB
#
# =================================================================================================
function dn::callback_execute_compose_post() {

    if [[ ${DOCKER_COMPOSE_CMD_ARGS[0]:?err} == build ]] && [[ "${DOCKER_COMPOSE_CMD_ARGS[*]}" =~ .*"--push".* ]] && [[ ! "${DOCKER_COMPOSE_CMD_ARGS[*]}" =~ .*"--dry-run".* ]]; then
      n2st::print_msg "Preparing docker manifeste for multiarch image"

      docker push "${DN_HUB:?err}/dockerized-norlab-base-image:${DN_IMAGE_TAG:?err}-arm64"
      docker push "${DN_HUB}/dockerized-norlab-base-image:${DN_IMAGE_TAG}-amd64"

      docker buildx imagetools create --tag "${DN_HUB}/dockerized-norlab-base-image:${DN_IMAGE_TAG}" \
           "${DN_HUB}/dockerized-norlab-base-image:${DN_IMAGE_TAG}-arm64" \
           "${DN_HUB}/dockerized-norlab-base-image:${DN_IMAGE_TAG}-amd64"

      n2st::print_msg_done "Multiarch image pushed to docker registry"
    else
      n2st::print_msg_warning "Skip pushing dockerized-norlab-base-image multi-arch image"
    fi

    docker buildx imagetools inspect "${DN_HUB:?err}/dockerized-norlab-base-image:${DN_IMAGE_TAG:?err}" --raw

}
