
FROM osrf/ros2:bouncy-desktop
LABEL maintainer="alberto dot soragna at gmail dot com"

# working directory
ENV HOME /root
WORKDIR $HOME

ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# general utilities
RUN apt-get update && apt-get install -y \
    curl \
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
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget

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


######## TURTLEBOT 3 SETUP


# get ros2 turtlebot sources
RUN mkdir -p $HOME/turtlebot_ws/src
WORKDIR $HOME/turtlebot_ws/src
RUN git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3.git
RUN git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
RUN git clone -b ros2 https://github.com/ros2/cartographer.git
RUN git clone -b 2.1.1 https://github.com/ros2/cartographer_ros.git
RUN git clone https://github.com/ros2/pcl_conversions.git

# install cartographer dependencies
RUN apt-get install --no-install-recommends -y \
  libboost-iostreams-dev \
  libboost-regex-dev \
  libboost-system-dev \
  libboost-thread-dev \
  libceres-dev \
  libgoogle-glog-dev \
  liblua5.2-dev \
  libpcl-dev \
  libprotobuf-dev \
  libsdl1.2-dev \
  libsdl-image1.2-dev \
  libsuitesparse-dev \
  libudev-dev \
  libusb-1.0-0-dev \
  libyaml-cpp-dev \
  protobuf-compiler \
  python-sphinx \
  libcairo2-dev

# ugly bug fix https://github.com/ROBOTIS-GIT/turtlebot3/issues/308
# moves line 33 up to line 22 in file
#RUN (echo 33 m 22; echo wq) | ex cartographer/cartographer-config.cmake.in

# build turtlebot sources
WORKDIR $HOME/turtlebot_ws
RUN /bin/bash -c 'source /opt/ros/bouncy/setup.sh; \
  colcon build'

RUN echo ' \n\
echo "Sourcing ROS2 and Turtlebot packages..." \n\
source /opt/ros/bouncy/setup.sh \n\
source $HOME/turtlebot_ws/install/local_setup.sh ' >> $HOME/.bashrc
