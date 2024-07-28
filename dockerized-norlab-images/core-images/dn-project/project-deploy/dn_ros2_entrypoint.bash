#!/bin/bash
#set -e


source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash

cd "${DN_PROJECT_PATH:?"Env variable need to be set and non-empty"}" || exit 1

if [[ -f /project_entrypoints/dn_entrypoint.main.callback.bash ]]; then
  source /project_entrypoints/dn_entrypoint.main.callback.bash
fi

exec "$@"
