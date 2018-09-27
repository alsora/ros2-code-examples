# Compile ROS2 from sources

## System setup

Follow the instructions for System setup in [installing_ros2.md](https://github.com/alsora/ros2-tutorial/blob/master/installing_ros2.md)

### Cross-compile ROS2 for AArch64 (ARMv8) systems

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

