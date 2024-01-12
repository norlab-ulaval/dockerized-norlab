#!/bin/bash
#
# Convenient script
#
# Usage:
#   $ bash dn_attach.bash [<optional flag>] <container-name>
#
# Arguments:
#   You can pass any docker exec flag in <optional argument> eg.:
#     --env=\"VAR=1\"        (to set environment variables)
#

if [[ $( basename $(pwd) ) = run_script ]]; then
    cd ../..
elif [[ $( basename $(pwd) ) = dockerized-norlab-scripts ]]; then
    cd ..
fi

# ....Pre-condition.................................................................................
if [[ ! -f  ".env.dockerized-norlab" ]]; then
  echo -e "\n[\033[1;31mERROR\033[0m] 'dn_attach.bash' script must be executed from the project root!\n Curent working directory is '$(pwd)'"
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
## import shell functions from norlab-shell-script-tools utilities library

TMP_CWD=$(pwd)
# ToDo: refactor > use N2ST_PATH set somewhere
cd ./utilities/norlab-shell-script-tools/src/function_library
source ./prompt_utilities.bash
source ./terminal_splash.bash
cd "$TMP_CWD"
SP="    "

function print_help_in_terminal() {
  echo -e "\$ ${0} [<optional argument>] <THE_CONTAINER_NAME>

Open a new interactive terminal with pseudo-TTY
${SP}
${SP}\033[1m<optional argument>:\033[0m
${SP}  -h, --help                Get help
${SP}
\033[1mNote:\033[0m You can pass any docker build flag in <optional argument> eg.:

${SP}--env=\"VAR=1\"        (to set environment variables)

\033[2mRef. docker exec command:
  - https://docs.docker.com/engine/reference/commandline/exec/
\033[0m
"

  echo -e "Terminal prompt › The default Dockerized-NorLab prompt require that\033[1;37m Powerline-status\033[0m or\033[1;37m Powerline10k\033[0m be installed on the host terminal. To change to a minimal prompt, either set permanently the ENV variable in\033[1;37m docker-compose.<spec>.run.yaml\033[0m:
${SP}
${SP}services:
${SP}  develop: # the service name
${SP}\033[1;37m    environment:
${SP}      - DN_ACTIVATE_POWERLINE_PROMT=false
\033[0m
or pass the following flag to \033[1;37mdn_attach\033[0m when connecting to a running container:
\033[1;37m
${SP}$ dn_attach --env=\"DN_ACTIVATE_POWERLINE_PROMT=false\" <the-running-container-name>
\033[0m
"
}

## todo: on task end >> comment next dev bloc ↓↓
#echo "${0}: all arg >>" \
#  && echo "${@}"

if [ $# -lt 1 ]; then
  echo -e "${MSG_ERROR} Missing argument: $0 ${MSG_ERROR_FORMAT}<THE_CONTAINER_NAME>${MSG_END_FORMAT}
If your not sure, execute ${MSG_EMPH_FORMAT}\$ docker ps -a${MSG_END_FORMAT} in host terminal and check the STATUS column to see running container"
  exit 1
fi

THE_CONTAINER_NAME=""
USER_ARG=""

# (CRITICAL) ToDo: validate (ref task NMO-257 ♻︎ → dn_attach.bash command line flag logic)
for arg in "$@"; do
  case $arg in
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  --)
    shift
    ;;
  -?* | --?*)
    #    echo $0: $1: unrecognized option >&2 # Note: '>&2' = print to stderr
    USER_ARG="${USER_ARG} ${arg}"
    shift # Remove generic argument from processing
    ;;
  *)
    THE_CONTAINER_NAME="${arg}"
    break
    ;;
  esac

  shift
done

## todo:on task end >> delete next bloc ↓↓
#echo "
#${0}:
#  USER_ARG >> ${USER_ARG}
#  THE_CONTAINER_NAME >> ${THE_CONTAINER_NAME}
#"

# ....Display and xhost............................................................................
# (CRITICAL) ToDo: Check the Dusty-nv implementation for X11 forwarding (ref task NMO-183 Fix GUI display issue)

#export DISPLAY=:0
##echo "export DISPLAY=:0" >> ~/.bashrc
#
## Note on xhost usage:
##           $ xhost [[+-][family:]name]
##
##   familly:
##     - local:      contains only one name, the empty string
##     - inet:       Internet host (IPv4)
##     - inet6:      Internet host (IPv6)
##     - si:         Server Interpreted : si:<type>:<value>
#
#xhost +si:localuser:root
##sudo xhost + # (Priority) todo:fixme!!
##   (ref task NMO-87 🩹→ Find a secure and permanent solution for the xhost "display not available" problem)

# ====Begin=========================================================================================
n2st::norlab_splash "${DN_SPLASH_NAME}" "${PROJECT_GIT_REMOTE_URL}"

# Fetch all container name, strip those unrelated one and test for exact name
if [[ $(docker ps --all --format "{{.Names}} {{.State}}" | grep ${THE_CONTAINER_NAME}) == "${THE_CONTAINER_NAME} exited" ]]; then
    # Start container if he is stopped
    echo -e "${MSG_BASE} Starting container $(docker start ${THE_CONTAINER_NAME})"
    echo ""
elif [[ -z $(docker ps --all --format "{{.Names}}" | grep ${THE_CONTAINER_NAME}) ]]; then
    echo -e "${MSG_ERROR} Container ${THE_CONTAINER_NAME} is not instantiated."
#    "Use command ${MSG_BASE_FORMAT}dn_instantiate_develop${MSG_END_FORMAT}"
    exit
fi

#sudo docker exec -it ${USER_ARG} ${THE_CONTAINER_NAME} bash
docker exec -it ${USER_ARG} ${THE_CONTAINER_NAME} bash
