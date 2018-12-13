FROM ubuntu:16.04
LABEL maintainer="alberto dot soragna at gmail dot com"

# working directory
ENV HOME /root
WORKDIR $HOME

ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    vim \
    nano \
    python-pip \
    python-dev \
    lsb-release \
    sudo


RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full

RUN rosdep init
RUN rosdep update


RUN mkdir -p $HOME/turtlebot3_ws/src
WORKDIR $HOME/turtlebot3_ws/src
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
WORKDIR $HOME/turtlebot3_ws

RUN /bin/bash -c 'source /opt/ros/kinetic/setup.sh; \
  catkin_make'


RUN echo ' \n\
echo "Sourcing ROS2 and Turtlebot packages..." \n\
source /opt/ros/kinetic/setup.sh \n\
source $HOME/turtlebot3_ws/devel/setup.sh ' >> $HOME/.bashrc

