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
          --volume=/home/alsora/source/alsora/ros2-examples:/root/ros2-examples \
	  ros2_dev \
	  bash
