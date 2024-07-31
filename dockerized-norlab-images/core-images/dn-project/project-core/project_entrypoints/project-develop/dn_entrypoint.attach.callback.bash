#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container up_and_attach.bash entrypoint callback.
# Is executed each time a shell is attach to a project-develop container.
#
# Usage:
#   Add only project-develop specific logic that need to be executed by each shell.
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
set -e
