#!/bin/bash
#
# @author Alberto Soragna (alberto dot soragna at gmail dot com)
# @2018 


docker run -it --rm \
	  --net=host \
	  --privileged \
	  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
	  --device=/dev/dri:/dev/dri \
	  --env="DISPLAY" \
	  ros2_turtlebot \
	  bash
