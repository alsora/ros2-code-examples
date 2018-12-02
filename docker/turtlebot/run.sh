#!/bin/bash
#
# @author Alberto Soragna (alberto dot soragna at gmail dot com)
# @2018


XSOCK=/tmp/.X11-unix

docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v $XSOCK:$XSOCK \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $PWD/scripts:/root/scripts \
  --privileged \
  --net=host \
  ros2_turtlebot bash




