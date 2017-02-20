#!/bin/bash

CONTAINER=$1
IMAGE_NAME=$2
ADDITIONAL_DOCKER_ARGS=$3

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
REPO_DIR=`readlink -f ${SCRIPTS_DIR}/../`


DOCKER_MOUNT_ARGS="\
    -v ${HOME}/.tmux.conf:/home/gazebo/.tmux.conf \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v /dev:/dev"

xhost +
docker run --name ${1} --privileged --rm \
    ${DOCKER_MOUNT_ARGS} \
    -e DISPLAY=${DISPLAY} ${ADDITIONAL_DOCKER_ARGS} \
    --net=host \
    --device /dev/dri \
    -it ${2}
