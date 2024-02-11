#!/bin/bash

function dn::callback_execute_compose_post() {

    n2st::print_msg "Preparing docker manifeste for multiarch image"

    docker manifest create --amend "${DN_HUB:?err}/dockerized-norlab-base-image:${DN_IMAGE_TAG:?err}" \
         "${DN_HUB}/dockerized-norlab-base-image:${DN_IMAGE_TAG}-arm64" \
         "${DN_HUB}/dockerized-norlab-base-image:${DN_IMAGE_TAG}-amd64"

    docker manifest push "${DN_HUB}/dockerized-norlab-base-image:${DN_IMAGE_TAG}"

    n2st::print_msg_done "Multiarch image push to docker registry"

}
