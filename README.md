# ros2-code-examples

The following tutorials and examples are updated for **ROS 2 Bouncy Bolson** (Released on 2 July 2018).

## Requirements

 - [ROS2 Bouncy](https://index.ros.org/doc/ros2/Installation/)


## Building the examples

Source your ROS2 SDK, then create a workspace and build these packages

    $ mkdir -p ws/src
    $ cd ws/src
    $ git clone https://github.com/alsora/ros2-code-examples --recurse-submodules
    $ cd ..    
    $ colcon build
    $ source install/local_setup.sh

For more detailed build instructions look [here](build_ros2_packages.md)


## Running the examples

For each of the following tests, run the each command in a separate terminal window (remember that ROS2 SDK has to be sourced in every terminal!)

#### Publisher/Subscriber test
```
    $ ros2 run simple_publisher publisher_main
```
```
    $ ros2 run simple_subscriber subscriber_main
```
#### Service/Client test
```
    $ ros2 run simple_service service_main
```
```
    $ ros2 run simple_client client_main
```
#### Time Synchronization message filters test
```
    $ ros2 run simple_time_sync publisher
```
```
    $ ros2 run simple_time_sync time_sync_main
```
#### Logger utils
```
    $ ros2 run simple_logger logger_main
```
#### Parameter servers
```
    $ ros2 run simple_parameters parameters_main
```
```
    $ ros2 run simple_parameters reader_main
```
#### Multiple nodes pub/sub
```
    $ ros2 run simple_multithread simple_pub_sub_std
```
## ROS2 CLI (command line interface)

Note that these commands comes from a Python package. So if you have disabled them (i.e. when cross-compiling) they will not be available.

To see a list of available commands

    $ ros2 --help

Print a list of <package_name> <executable_name>

    $ ros2 pkg executabels

Run a ROS2 node

    $ ros2 run <package_name> <executable_name>
