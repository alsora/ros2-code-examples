
FROM osrf/ros2:bouncy-desktop
LABEL maintainer="alberto dot soragna at gmail dot com"

# working directory
ENV HOME /root
WORKDIR $HOME

# general utilities
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    vim \
    nano \
    python3-pip

# pip
RUN pip3 install --upgrade pip

# Locale options
RUN apt-get install -y \
    locales

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

RUN python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures

# install Fast-RTPS dependencies
RUN apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev \
  libopensplice67

# dependencies for RViz
RUN apt-get install -y \
  libfreetype6-dev \
  libfreeimage-dev \
  libzzip-dev \
  libxrandr-dev \
  libxaw7-dev \
  freeglut3-dev \
  libgl1-mesa-dev \
  libcurl4-openssl-dev \
  libqt5core5a \
  libqt5gui5 \
  libqt5opengl5 \
  libqt5widgets5 \
  libxaw7-dev \
  libgles2-mesa-dev \
  libglu1-mesa-dev \
  qtbase5-dev

# source everything
RUN echo ' \n\
echo "Sourcing ROS2 packages..." \n\
source /opt/ros/bouncy/setup.sh ' >> $HOME/.bashrc

