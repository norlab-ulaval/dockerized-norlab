#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container entrypoint callback
# Is executed at the end of the project-develop and project-deploy entrypoints
#
# Usage:
#   Just add you code
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
set -e


# Sanity check
test -n "$(pgrep -x 'sshd')" || echo "ssh daemon is not running!"
