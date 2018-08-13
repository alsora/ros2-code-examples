#!/bin/bash
#
# @author Alberto Soragna (alberto dot soragna at gmail dot com)
# @2018 


XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it --rm \
	     --net=host \
	     -v $XSOCK:$XSOCK:rw \
	     -v $XAUTH:$XAUTH:rw \
	     -e XAUTHORITY=${XAUTH} \
             -e DISPLAY=$DISPLAY \
             -e QT_GRAPHICSSYSTEM=native \
	     --name ros \
	     ros2_docker \
	     bash
