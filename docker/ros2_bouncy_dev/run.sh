#!/bin/bash
#
# @author Alberto Soragna (alberto dot soragna at gmail dot com)
# @2018 

XSOCK=/tmp/.X11-unix

docker run -it --rm \
	  --net=host \
	  --privileged \
	  -e DISPLAY=$DISPLAY \
	  -v $XSOCK:$XSOCK \
	  -v $HOME/.Xauthority:/root/.Xauthority \
          --volume=$PWD/../../:/root/ros2-code-examples \
	  ros2_bouncy_dev \
	  bash
