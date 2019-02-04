# simple_gazebo

ROS2 Crystal Clemmys introduces support for the 3D Simulation and Visualization tool Gazebo 9.

Differences with the ROS1 version [here](http://gazebosim.org/tutorials?tut=ros2_overview&branch=ros2)

## Install

Install Gazebo
[More detailed instructions](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1)

    $ wget https://bitbucket.org/osrf/release-tools/raw/default/one-line-installations/gazebo.sh
    $ chmod +x gazebo.sh
    $ ./gazebo.sh


Test if Gazebo is working

    $ gazebo

### Install from debian packages

If you are installing ROS2 from debian packages, you can easily do the same for gazebo.

    $ sudo apt install -y  ros-crystal-vision-opencv ros-crystal-gazebo-ros-pkgs

### Build from sources

On the other hand, if you are building from sources

Create a workspace for gazebo_ros_pkgs

    $ mkdir -p ~/gazebo_ws/src
    $ cd ~/gazebo_ws/src
    $ git clone -b ros2 https://github.com/ros-simulation/gazebo_ros_pkgs
    $ git clone -b ros2 https://github.com/ros-perception/vision_opencv
    $ git clone -b ros2 https://github.com/ros-perception/image_common

Install other dependencies

    $ cd ..
    $ rosdep install --from-paths src --ignore-src -r -y

Build the workspace and source it

    $ colcon build
    $ source install/local_setup.sh


## Usage

Now everything should be ready.
You can run a Gazebo world file, as the example provided in the gazebo plugins package

    $ gazebo --verbose /opt/ros/crystal/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
    
Now you should be able to see the ROS2 topics used by this example

    $ ros2 topic list

For example, to send a velocity command to the robot

    $ ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1
    
If you want to change the settings for this simulation example, just edit its related world file

    $ gedit /opt/ros/crystal/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
