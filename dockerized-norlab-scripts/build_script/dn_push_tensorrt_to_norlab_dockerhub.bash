#!/bin/bash
# =================================================================================================
# Pull TensorRT images from Nvidia hub and push them to NorLab dockerhub domain.
# ★ Run it manualy to generate missing image tag
#
# 🪴️ Why: Mitigate the 'unauthorized access' error which fail the build from time to time.
#
# Required by 'dockerized-norlab-images/core-images/base-images/l4t-images/' images:
#   - 'dn_callback_execute_compose_pre.bash' script (see 'MIMIC_DEPENDENCIES_BASE_IMAGE')
#   - and stage 'base-image-amd64' and 'mimic-l4t-image' of 'Dockerfile.l4t.squash'
#
# Usage:
#   $ bash dockerized-norlab-scripts/build_script/dn_push_tensorrt_to_norlab_dockerhub.bash
#
# Reference of key base images with full tag:
#   - dustynv/ros:foxy-pytorch-l4t-r35.2.1
#   - dustynv/pytorch:2.1-r35.2.1
#   - nvcr.io/nvidia/l4t-jetpack:r35.2.1
#   - nvcr.io/nvidia/l4t-base:r35.2.1
#   - nvidia/cuda:12.3.1-runtime-ubuntu20.04 (ref https://hub.docker.com/r/nvidia/cuda)
#   - nvcr.io/nvidia/tensorrt:20.12-py3 (https://catalog.ngc.nvidia.com/orgs/nvidia/containers/tensorrt/tags)
#   - nvcr.io/nvidia/tensorrt:YY.MM-py3 with YY=year and MM=month
#     TensorRT Release 23.04 -> last Ubuntu 20.04 release -> python 3.8
#       https://docs.nvidia.com/deeplearning/frameworks/container-release-notes/index.html#rel-23-04
#     TensorRT Release 24.10 -> last Ubuntu 22.04 release -> python 3.10
#       https://docs.nvidia.com/deeplearning/frameworks/container-release-notes/index.html#rel-24-10)
#
# =================================================================================================

TENSORRT_TAG=(
  "23.04-py3"
  "24.10-py3"
  "25.06-py3"
)

for tag in "${TENSORRT_TAG[@]}"; do
  # Pull images from Nvidia domain
  docker pull --platform=linux/amd64 "nvcr.io/nvidia/tensorrt:${tag}" || { echo "‼️" 1>&2 ; exit 1 ; }

  # Retag them
  docker tag "nvcr.io/nvidia/tensorrt:${tag}" "norlabulaval/nvidia-tensorrt:${tag}"

  # Push to NorLab domain
  docker push "norlabulaval/nvidia-tensorrt:${tag}" || { echo "‼️" 1>&2 ; exit 1 ; }

  echo -e "\nnvcr.io/nvidia/tensorrt:${tag}: -> norlabulaval/nvidia-tensorrt:${tag} done\n"
done

echo "✅ All images where pushed to https://hub.docker.com/repository/docker/norlabulaval/nvidia-tensorrt/general
"

exit 0
