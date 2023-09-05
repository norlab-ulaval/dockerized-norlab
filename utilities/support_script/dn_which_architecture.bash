#!/bin/bash

set -e

# Load environment variable from file
#set -o allexport; source "${DN_PATH}/.env.prompt"; set +o allexport
set -o allexport; source .env.prompt; set +o allexport

# DN_IMAGE_ARCHITECTURE: arm64-l4t, arm64-darwin, x86
# ARCH: aarch64, arm64, x86_64
# OS: Linux, Darwin

if [[ $(uname -m) == "aarch64" ]]; then
  if [[ -n $(uname -r | grep tegra) ]]; then
    export DN_IMAGE_ARCHITECTURE='arm64-l4t'
  else
    echo -e "${MSG_ERROR} Unsuported OS for aarch64 processor"
  fi
elif [[ $(uname -m) == "arm64" ]]; then
  if [[ $(uname) == "Darwin" ]]; then
    export DN_IMAGE_ARCHITECTURE='arm64-darwin'
  else
    export DN_IMAGE_ARCHITECTURE='arm64' # ToDo: validate
  fi
elif [[ $(uname -m) == "x86_64" ]]; then
  if [[ $(uname) == "Linux" ]]; then
    export DN_IMAGE_ARCHITECTURE='x86'
  else
    echo -e "${MSG_ERROR} Unsuported OS for x86 processor"
  fi
else
  echo -e "${MSG_ERROR} Unsuported processor architecture"
fi

echo -e "${MSG_BASE} which DN_IMAGE_ARCHITECTURE env? $DN_IMAGE_ARCHITECTURE"
