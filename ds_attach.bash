#!/bin/bash

# Load environment variable from file
set -o allexport; source ds.env; set +o allexport


bash ./visual/terminal_splash.bash

function print_help_in_terminal() {

  echo -e "\$ ${0} [<optional argument>] <CONTAINER_NAMES>

Open a new interactive terminal with pseudo-TTY

\033[1m<optional argument>:\033[0m
  -h, --help                Get help

\033[1mNote:\033[0m You can pass any docker build flag in <optional argument> eg.:
  --env=\"VAR=1\"        (to set environment variables)

\033[2mRef. docker exec command:
  - https://docs.docker.com/engine/reference/commandline/exec/
\033[0m"
}

## todo:on task end >> delete next bloc ↓↓
#echo "
#${0}: all arg >> ${@}
#"

if [ $# -ne 1 ]; then
  echo -e "${DS_MSG_ERROR} Missing argument: $0 ${DS_MSG_ERROR_FORMAT}<CONTAINER_NAMES>${DS_MSG_END_FORMAT}
If your not sure, execute ${DS_MSG_EMPH_FORMAT}\$ docker ps -a${DS_MSG_END_FORMAT} in host terminal and check the STATUS column to see running container"
  exit 1
fi

CONTAINER_NAMES=""
USER_ARG=""

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


# Fetch all container name, strip those unrelated one and test for exact name
if [[ `docker ps --quiet --all --format "{{.Names}} {{.State}}" | grep ${CONTAINER_NAMES}` == "${CONTAINER_NAMES} exited" ]]; then
    # Start container if he is stopped
    echo -e "${DS_MSG_BASE} Starting container $(docker start ${CONTAINER_NAMES})"
    echo ""
elif [[ -z `docker ps --quiet --all --format "{{.Names}}" | grep ${CONTAINER_NAMES}` ]]; then
    echo -e "${DS_MSG_ERROR} Container ${CONTAINER_NAMES} is not instantiated. Use command ${DS_MSG_BASE_FORMAT}ds_instantiate_develop${DS_MSG_END_FORMAT}"
    exit
fi

sudo docker exec -it ${USER_ARG} ${CONTAINER_NAMES} bash
