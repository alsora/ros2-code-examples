# ros2-examples

The following tutorials and examples are updated for **ROS 2 Bouncy Bolson** (2 July 2018).

- [Why moving to ROS 2 ?](http://design.ros2.org/articles/why_ros2.html)
- [Installing ROS 2](https://github.com/ros2/ros2/wiki/Installation) 
- [Building a new package in ROS 2](https://github.com/ros2/ros2/wiki/Migration-Guide)
- [Converting a ROS package to ROS 2](https://github.com/ros2/ros2/wiki/Migration-Guide)
- [Cross-compiling ROS 2 for arm architectures](https://github.com/ros2-for-arm/ros2/wiki/ROS2-on-arm-architecture)
- [ROS in Docker](https://jarisafi.wordpress.com/2018/01/17/how-i-use-docker-for-robotics-development/)


### Installation

Build a Docker image and deploy a container

    $ cd docker
    $ bash build.sh
    $ bash run.sh

Inside the Docker container, to see a list of available commands

    # ros2 --help

Check if Rviz GUI is properly displayed

    # ros2 run rviz2
    
Run nodes inside the Docker container 

    # ros2 run demo_nodes_cpp listener & ros2 run demo_nodes_cpp talker

[How to run nodes in separate Docker containers](https://github.com/ros2/ros2/wiki/Run-2-nodes-in-two-separate-docker-containers)


