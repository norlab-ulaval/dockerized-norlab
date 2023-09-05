#!/bin/bash

# Load environment variable from file
set -o allexport; source .env.prompt; set +o allexport


bash ./visual/terminal_splash.bash

SP="    "

function print_help_in_terminal() {

  echo -e "\$ ${0} [<optional argument>] <CONTAINER_NAMES>

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
  echo -e "${MSG_ERROR} Missing argument: $0 ${MSG_ERROR_FORMAT}<CONTAINER_NAMES>${MSG_END_FORMAT}
If your not sure, execute ${MSG_EMPH_FORMAT}\$ docker ps -a${MSG_END_FORMAT} in host terminal and check the STATUS column to see running container"
  exit 1
fi

CONTAINER_NAMES=""
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
    CONTAINER_NAMES="${arg}"
    break
    ;;
  esac

  shift
done

## todo:on task end >> delete next bloc ↓↓
#echo "
#${0}:
#  USER_ARG >> ${USER_ARG}
#  CONTAINER_NAMES >> ${CONTAINER_NAMES}
#"


# todo:on task end >> delete next bloc ↓↓
#if [[ `docker ps --quiet --all --format "{{.Names}} {{.State}}" | grep ${CONTAINER_NAMES}` == "${CONTAINER_NAMES} exited" ]]; then echo "if TRUE"; else echo "else FALSE"; fi
#if [[ -z `docker ps --quiet --all --format "{{.Names}}" | grep IamNOTsnow` ]]; then echo "if TRUE"; else echo "else FALSE"; fi

# ››› Display and xhost ››› . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# (CRITICAL) ToDo: Check the Dusty-nv implementation for X11 forwarding (ref task NMO-183 Fix GUI display issue)

export DISPLAY=:0
#echo "export DISPLAY=:0" >> ~/.bashrc

# Note on xhost usage:
#           $ xhost [[+-][family:]name]
#
#   familly:
#     - local:      contains only one name, the empty string
#     - inet:       Internet host (IPv4)
#     - inet6:      Internet host (IPv6)
#     - si:         Server Interpreted : si:<type>:<value>

xhost +si:localuser:root
#sudo xhost + # (Priority) todo:fixme!!
#   (ref task NMO-87 🩹→ Find a secure and permanent solution for the xhost "display not available" problem)
#  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .‹‹‹ Display and xhost ‹‹‹

# Fetch all container name, strip those unrelated one and test for exact name
if [[ `docker ps --quiet --all --format "{{.Names}} {{.State}}" | grep ${CONTAINER_NAMES}` == "${CONTAINER_NAMES} exited" ]]; then
    # Start container if he is stopped
    echo -e "${MSG_BASE} Starting container $(docker start ${CONTAINER_NAMES})"
    echo ""
elif [[ -z `docker ps --quiet --all --format "{{.Names}}" | grep ${CONTAINER_NAMES}` ]]; then
    echo -e "${MSG_ERROR} Container ${CONTAINER_NAMES} is not instantiated."
#    "Use command ${MSG_BASE_FORMAT}dn_instantiate_develop${MSG_END_FORMAT}"
    exit
fi

#sudo docker exec -it ${USER_ARG} ${CONTAINER_NAMES} bash
docker exec -it ${USER_ARG} ${CONTAINER_NAMES} bash
