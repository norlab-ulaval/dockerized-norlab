#!/bin/bash
#set -e


source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash

cd "${DN_PROJECT_PATH:?"Env variable need to be set and non-empty"}" || exit 1

if [[ -f /dn_entrypoint.project.callback.bash ]]; then
  source /dn_entrypoint.project.callback.bash
fi

exec "$@"
