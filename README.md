# ros2-code-examples

The following tutorials and examples are updated for **ROS 2 Bouncy Bolson** (Released on 2 July 2018).


## Building the examples

Source your ROS2 SDK, then create a workspace and build these packages

    $ mkdir -p ws/src
    $ cd ws/src
    $ git clone https://github.com/alsora/ros2-code-examples
    $ cd ..    
    $ colcon build
    $ source install/local_setup.sh


## Running the examples

Run the following command each in a separate terminal

#### Publisher/Subscriber test

    $ ros2 run simple_publisher publisher_main
    $ ros2 run simple_subscriber subscriber_main

#### Service/Client test

    $ ros2 run simple_service service_main
    $ ros2 run simple_client client_main

Optional: Run also Rviz to visualize the data 

    $ rviz2

#### Time Synchronization message filters test

    $ ros2 run simple_time_sync publisher
    $ ros2 run simple_time_sync time_sync_main

#### Logger utils

    $ ros2 run simple_logger logger_main

#### Parameter servers

    $ ros2 run simple_parameters parameters_main
    $ ros2 run simple_parameters reader_main

#### Multiple nodes pub/sub

    $ ros2 run simple_multithread simple_pub_sub_std

## ROS2 CLI (command line interface)

Note that these commands comes from a Python package. So if you have disabled them (i.e. when cross-compiling) they will not be available.

To see a list of available commands

    $ ros2 --help

Print a list of <package_name> <executable_name>

    $ ros2 pkg executabels

Run a ROS2 node

    $ ros2 run <package_name> <executable_name>
