# Compile ROS2 from sources

## System setup


Install system requirements
```
$ sudo apt-get update && sudo apt-get install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8
```

Setup souces
```
$ sudo apt-get install curl
$ curl http://repo.ros2.org/repos.key | sudo apt-key add -
$ sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
```

Install development tools and ROS tools

```
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget
# install some pip packages needed for testing
sudo -H python3 -m pip install -U \
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
# [Ubuntu 16.04] install extra packages not available or recent enough on Xenial
python3 -m pip install -U \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
```

Create a workspace and get the ROS2 source code
```
$ mkdir -p ros2_ws/src
$ cd ros2_ws
$ wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
$ vcs-import src < ros2.repos
$ vcs-import src < ros2-for-arm.repos
```

### Compile ROS2 for your host system

Install dependencies using rosdep

```
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"
```

Build the code in the workspace

```
colcon build --symlink-install
```
