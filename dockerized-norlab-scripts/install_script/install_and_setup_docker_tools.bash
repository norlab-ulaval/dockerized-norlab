#!/bin/bash
#
# Convenient script for installing docker related tools
#
# usage:
#   $ bash ./install_script/install_and_setup_docker_tools.bash [<optional flag>]
#
# Arguments:
#   - [<optional flag>]   Any optional flag ...
#
#set -e

# ....Project root logic...........................................................................................
TMP_CWD=$(pwd)

if [[ "$(basename $(pwd))" = install_script ]]; then
  cd ../..
elif [[ "$(basename $(pwd))" = dockerized-norlab-scripts ]]; then
    cd ..
fi

# ....Pre-condition.................................................................................
if [[ ! -f  ".env.dockerized-norlab" ]]; then
  echo -e "\n[\033[1;31mERROR\033[0m] 'install_and_setup_docker_tools.bash' script should be sourced from its parent directory or the project root!\n Curent working directory is '$(pwd)'"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ....Load environment variables from file....................................................................
set -o allexport
source .env.dockerized-norlab-project
set +o allexport

set -o allexport
# ToDo: refactor > use N2ST_PATH set somewhere
source ./utilities/norlab-shell-script-tools/.env.project
set +o allexport

# ....Helper function...............................................................................

TMP_CWD=$(pwd)
# ToDo: refactor > use N2ST_PATH set somewhere
cd ./utilities/norlab-shell-script-tools/src/function_library
source ./prompt_utilities.bash
source ./terminal_splash.bash
cd "$TMP_CWD"

# ====Begin========================================================================================================
n2st::print_formated_script_header 'install_and_setup_docker_tools.bash' '='

n2st::print_msg_awaiting_input "Do you want to install Docker tools (Linux)?"
echo
TMP_MSG="(press 'Y' to install, or press any other key to skip) "
echo_centering_str "${TMP_MSG}" "\033[2m" " "
echo
read -n 1 -r -a INPUT

if [[ ${INPUT} == "Y" ]] || [[ ${INPUT} == "y" ]]; then
  cd .utilities/norlab-shell-script-tools/src/utility_scripts/
  bash install_docker_tools.bash
  cd "$TMP_CWD"
fi
unset INPUT
echo
n2st::print_msg "Current available docker context"
echo
docker context ls
echo
echo
n2st::print_msg "Current available docker builder instance"
echo
docker buildx ls
echo

n2st::print_msg_awaiting_input "Do you want to configure a docker multi-arch builder instance?"
echo
TMP_MSG="(press 'Y' to install, or press any other key to skip)"
echo_centering_str "${TMP_MSG}"  "\033[2m" " "
echo
read -n 1 -r -a INPUT
echo
if [[ ${INPUT} == "Y" ]] || [[ ${INPUT} == "y" ]]; then
  n2st::print_msg "Create a new docker builder instance and context"

  # ..................................................................................................
#  docker buildx rm multi-arch-builder
#  docker context rm jetson-nano-1
#  docker buildx rm remote-builder-jetson-nx-redleader

  # ..................................................................................................
  docker buildx create \
    --name multi-arch-builder \
    --driver docker-container \
    --node local \
    --platform linux/amd64,linux/arm64 \
    --bootstrap \
    --use

#  # ..................................................................................................
#  # Note: flag pattern: --docker "host=ssh://username@host:port"
#  docker context create \
#    --docker "host=ssh://snow@169.254.205.89:22" \
#    --description "Execute docker command remotly using the jetson docker deamon " \
#    jetson-nano-1


  ## Note: this setup running the builder inside a docker-container does not work for pushing
  ##      image to dockerhub registry when there is a apple computer in the loop for mysterious
  ##      reason. We leave the next commented code bloc for documentation purpose of the failled
  ##      attempt to make it work.
  ##      The solution with osX is maybe to create a custom network with port mapping and
  ##      than use the flag: --driver-opt "network=myCustomOSXNetwork"
  ##        (see https://docs.docker.com/build/drivers/docker-container/#custom-network)
  ##      However, using the remote docker builder through the jetson-nano-1 docker context works.
  ##
  #  docker buildx create \
  #    --name remote-builder-jetson-nx-redleader \
  #    --driver docker-container \
  #    --platform linux/arm64 \
  #    --driver-opt "network=host" \
  #    --buildkitd-flags "--allow-insecure-entitlement network.host --allow-insecure-entitlement security.insecure" \
  #    --bootstrap \
  #    jetson-nano-1
  #


  echo
  n2st::print_msg "Current available docker context"
  echo
  docker context ls
  echo
  echo
  n2st::print_msg "Current available docker builder instance"
  echo
  docker buildx ls
  echo

  cd "${TMP_CWD}"
fi


n2st::print_formated_script_footer 'install_and_setup_docker_tools.bash' '='
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"
