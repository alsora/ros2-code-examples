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

The steps up to this point are required both if you want to compile ROS2 on your system as well as if you want to cross-compile it for a target ARM system.

The next steps on the other hand are platform dependent. 

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


### Cross-compile ROS2 for ARM systems

In the following will be presented the steps for cross-compiling for an aarch64 system.

Get a compiler and a *toolchainfile.cmake file for your target system.


```
$ sudo apt install g++-aarch64-linux-gnu gcc-aarch64-linux-gnu
$ export CROSS_COMPILE=aarch64-linux-gnu-\
$ wget https://raw.githubusercontent.com/ros2-for-arm/ros2/master/aarch64_toolchainfile.cmake
```

Remove Python support and ignore optional packages

```
$ sed -i -r \
  's/<build(.+?py.+?)/<\!\-\-build\1\-\->/' \
  src/ros2/rosidl_defaults/rosidl_default_generators/package.xml

$ touch \
  src/ros-perception/laser_geometry/COLCON_IGNORE \
  src/ros/resource_retriever/COLCON_IGNORE \
  src/ros2/geometry2/COLCON_IGNORE \
  src/ros2/urdf/COLCON_IGNORE \
  src/ros2/demos/COLCON_IGNORE \
  src/ros2/kdl_parser/COLCON_IGNORE \
  src/ros2/ros1_bridge/COLCON_IGNORE \
  src/ros2/rmw_connext/COLCON_IGNORE \
  src/ros2/orocos_kinematics_dynamics/COLCON_IGNORE \
  src/ros2/examples/rclpy/COLCON_IGNORE \
  src/ros2/robot_state_publisher/COLCON_IGNORE \
  src/ros2/rviz/COLCON_IGNORE \
  src/ros2/rcl/rcl/test/COLCON_IGNORE \
  src/ros2/urdfdom/COLCON_IGNORE \
  src/ros2/rclpy/COLCON_IGNORE \
  src/ros2/rosidl_typesupport_opensplice/COLCON_IGNORE \
  src/ros2/system_tests/COLCON_IGNORE \
  src/ros2/rosidl_python/COLCON_IGNORE \
  src/ros2/rmw_opensplice/COLCON_IGNORE \
  src/ros2/rosidl_typesupport_connext/COLCON_IGNORE \
  src/ros2/rcl_interfaces/test_msgs/COLCON_IGNORE

```

Build specifying the toolchain of the target system
```
colcon build \
  --merge-install \
  --executor sequential \
  --cmake-force-configure \
  --cmake-args \
    --no-warn-unused-cli \
    -DCMAKE_TOOLCHAIN_FILE=`pwd`/aarch64_toolchainfile.cmake \
    -DTHIRDPARTY=ON \
    -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
    -DBUILD_TESTING:BOOL=OFF
```

Move the cross-compiled sdk to the target system

You need the address of the board. To get it open a terminal in the board and run

    # hostname
    
Copy the installed libraries

    $ rsync -Lvr --rsh=ssh install/lib root@<BOARD_ADDRESS>:/usr/lib

