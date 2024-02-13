#!/bin/bash

function dn::callback_execute_compose_post() {

    n2st::print_msg "Preparing docker manifeste for multiarch image"

    ## (CRITICAL) ToDo: on task end >> mute next bloc ↓↓
    #DN_HUB=norlabulaval
    #DN_IMAGE_TAG=DN-hot-l4t-pytorch-r35.2.1

    docker push "${DN_HUB}/dockerized-norlab-base-image:${DN_IMAGE_TAG}-arm64"
    docker push "${DN_HUB}/dockerized-norlab-base-image:${DN_IMAGE_TAG}-amd64"

    docker buildx imagetools create --tag "${DN_HUB:?err}/dockerized-norlab-base-image:${DN_IMAGE_TAG:?err}" \
         "${DN_HUB}/dockerized-norlab-base-image:${DN_IMAGE_TAG}-arm64" \
         "${DN_HUB}/dockerized-norlab-base-image:${DN_IMAGE_TAG}-amd64"

    docker buildx imagetools inspect "${DN_HUB:?err}/dockerized-norlab-base-image:${DN_IMAGE_TAG:?err}" --raw

    n2st::print_msg_done "Multiarch image pushed to docker registry"
}
