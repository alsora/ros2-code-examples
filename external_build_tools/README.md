# Using ROS2 with external build tools

If you want to include ROS2 functionalities into a non-ROS2 project, the build process could become not trivial.

Here you find examples for building an hello-ros executable using standard building tools (i.e. **CMake** and **SCons**) instead of the native ros one (**Colcon**).


Additional examples can be found in this ROS1 repository ([link](https://github.com/gerkey/ros1_external_use)).


## CMake

Building with CMake is almost straight-forward as the native tool ament-cmake, which is used for building the ros sdk, automatically generate CMake config files.


Build 

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    
Run

    $ ./hello_ros
    

## SCons

Building with SCons requires the knowledge of the dependent packages include and library directories paths.
They can be queried using the commandline CMake find-package command

To see a list of installed packages

    $ ros2 pkg list
    
For example, for the base package rclcpp.
    
To get the include paths 

    $ cmake --find-package -DNAME=rclcpp -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=COMPILE
    
To get the library paths
   
    $ cmake --find-package -DNAME=rclcpp -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=LINK





Build

    $ scons
    
Run

    $ ./hello_ros
    
