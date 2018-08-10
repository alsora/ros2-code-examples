
FROM osrf/ros2:bouncy-full
MAINTAINER Alberto Soragna alberto dot soragna at gmail dot com

# working directory
ENV HOME /root
WORKDIR $HOME

RUN mkdir -p ros2_ws/src

RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    vim \
    nano \
    locales \
    python3-pip

# pip
RUN pip3 install --upgrade pip

# Locale options
RUN locale-gen en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8
    

# ROS setup requirements
RUN apt-get install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  python-rosdep \
  python3-vcstool

# install Fast-RTPS dependencies
RUN apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

# dependencies for RViz
RUN apt-get install -y libfreetype6-dev libfreeimage-dev libzzip-dev libxrandr-dev libxaw7-dev freeglut3-dev libgl1-mesa-dev libcurl4-openssl-dev libqt5core5a libqt5gui5 libqt5opengl5 libqt5widgets5 libxaw7-dev libgles2-mesa-dev libglu1-mesa-dev qtbase5-dev






